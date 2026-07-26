/**
 * @file lv_aoostar.c
 *
 * AOOSTAR GEM12 / WTR MAX secondary screen display driver.
 *
 * Protocol (reverse-engineered by zehnm/aoostar-rs):
 *   AA 55 AA 55 [cmd 4B] [optional offset 4B LE] [data]
 *
 * Pixel data: RGB565 little-endian
 * Transfer: frame_start + chunk_packets + frame_end, single write()
 */

/*********************
 *      INCLUDES
 *********************/
#include "lv_aoostar.h"

#if LV_USE_AOOSTAR

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <linux/serial.h>

#include "../../../display/lv_display_private.h"
#include "../../../misc/lv_types.h"
#include "../../../misc/lv_log.h"

/*********************
 *      DEFINES
 *********************/

/* Display geometry */
#define AOOSTAR_HOR_RES         960
#define AOOSTAR_VER_RES         376
#define AOOSTAR_BPP             2               /* RGB565 */
#define AOOSTAR_FRAME_BYTES     (AOOSTAR_HOR_RES * AOOSTAR_VER_RES * AOOSTAR_BPP)  /* 721920 */

/* Serial protocol */
#define AOOSTAR_BAUDRATE        1500000
#define AOOSTAR_CHUNK_BYTES     48              /* confirmed working */
#define AOOSTAR_CHUNKS_PER_ROW  (AOOSTAR_HOR_RES * AOOSTAR_BPP / AOOSTAR_CHUNK_BYTES)  /* 40 */
#define AOOSTAR_TOTAL_CHUNKS    (AOOSTAR_VER_RES * AOOSTAR_CHUNKS_PER_ROW)             /* 15040 */

/* Chunk packet = sync(4) + cmd(4) + offset(4) + data(48) = 60 bytes */
#define AOOSTAR_CHUNK_PACKET_SZ (4 + 4 + 4 + AOOSTAR_CHUNK_BYTES)

/* DMA buffer capacity: frame_start + all chunks + frame_end */
#define AOOSTAR_DMA_BUF_SIZE    (16 + AOOSTAR_TOTAL_CHUNKS * AOOSTAR_CHUNK_PACKET_SZ + 8)

/* Pixel-to-chunk mapping helpers */
#define AOOSTAR_PIX2CHUNK(x)       ((x) / 24)
#define AOOSTAR_ROW_FIRST(y)       ((uint32_t)(y) * AOOSTAR_CHUNKS_PER_ROW)
#define AOOSTAR_ROW_LAST(y, x2)    (AOOSTAR_ROW_FIRST(y) + ((x2) * 2 + 1) / AOOSTAR_CHUNK_BYTES)

/**********************
 *      TYPEDEFS
 **********************/

/* Protocol static templates */
static const uint8_t aoostar_sync[4]      = {0xAA, 0x55, 0xAA, 0x55};
static const uint8_t aoostar_cmd_on[4]    = {0x0B, 0x00, 0x00, 0x00};
static const uint8_t aoostar_cmd_off[4]   = {0x0A, 0x00, 0x00, 0x00};
static const uint8_t aoostar_cmd_chunk[4] = {0x08, 0x00, 0x00, 0x00};

static const uint8_t aoostar_frame_start[16] = {
    0xAA, 0x55, 0xAA, 0x55,
    0x05, 0x00, 0x00, 0x00,
    0x04, 0x00, 0x0F, 0x2F,
    0x00, 0x04, 0x0B, 0x00,
};

static const uint8_t aoostar_frame_end[8] = {
    0xAA, 0x55, 0xAA, 0x55,
    0x06, 0x00, 0x00, 0x00,
};

typedef struct {
    int        fd;              /* serial port fd, -1 if not open */
    uint8_t  * prev_frame;      /* previous frame snapshot (721920 B) */
    uint8_t  * dma_buf;         /* DMA-friendly encode buffer */
    size_t     dma_buf_size;    /* allocated size of dma_buf */
} aoostar_drv_t;

/**********************
 *  STATIC PROTOTYPES
 **********************/

static int  _serial_open(const char * device);
static void _serial_close(aoostar_drv_t * drv);
static void _send_on(aoostar_drv_t * drv);
static void _send_off(aoostar_drv_t * drv);

static size_t _build_dma_buf(aoostar_drv_t * drv,
                              const lv_area_t * area,
                              const uint8_t * px_map);

static void flush_cb(lv_display_t * disp, const lv_area_t * area, uint8_t * px_map);
static void del_event_cb(lv_event_t * e);

/**********************
 *   STATIC FUNCTIONS — Serial
 **********************/

static int _serial_open(const char * device)
{
    int fd = open(device, O_RDWR | O_NOCTTY | O_SYNC);
    if(fd < 0) {
        LV_LOG_ERROR("Failed to open %s: %s", device, strerror(errno));
        return -1;
    }

    /* Standard termios: raw mode, 8N1 */
    struct termios tio;
    memset(&tio, 0, sizeof(tio));
    if(tcgetattr(fd, &tio) != 0) {
        LV_LOG_ERROR("tcgetattr failed: %s", strerror(errno));
        close(fd);
        return -1;
    }
    cfmakeraw(&tio);
    tio.c_cflag |= CLOCAL | CREAD | CS8;
    tio.c_cflag &= ~(CSTOPB | PARENB);
    tio.c_cc[VMIN]  = 0;
    tio.c_cc[VTIME] = 1;

    /* Set custom baud rate via ASYNC_SPD_CUST + serial_struct */
    int baud = AOOSTAR_BAUDRATE;
    {
        struct serial_struct ss;
        if(ioctl(fd, TIOCGSERIAL, &ss) == 0) {
            ss.flags &= ~ASYNC_SPD_MASK;
            ss.flags |= ASYNC_SPD_CUST;
            ss.custom_divisor = ss.baud_base / baud;
            if(ioctl(fd, TIOCSSERIAL, &ss) == 0) {
                /* B38400 as placeholder for custom divisor */
                cfsetispeed(&tio, B38400);
                cfsetospeed(&tio, B38400);
                LV_LOG_INFO("Serial %s: %d baud (custom divisor)", device, baud);
            } else {
                LV_LOG_WARN("TIOCSSERIAL failed: %s", strerror(errno));
            }
        } else {
            LV_LOG_WARN("TIOCGSERIAL failed: %s", strerror(errno));
        }
    }

    if(tcsetattr(fd, TCSANOW, &tio) != 0) {
        LV_LOG_ERROR("tcsetattr failed: %s", strerror(errno));
        close(fd);
        return -1;
    }

    tcflush(fd, TCIOFLUSH);
    return fd;
}

static void _serial_close(aoostar_drv_t * drv)
{
    if(drv->fd >= 0) {
        close(drv->fd);
        drv->fd = -1;
    }
}

static void _send_on(aoostar_drv_t * drv)
{
    uint8_t buf[8];
    memcpy(buf,      aoostar_sync,    4);
    memcpy(buf + 4,  aoostar_cmd_on,  4);
    write(drv->fd, buf, sizeof(buf));
    usleep(500000);
    tcflush(drv->fd, TCIFLUSH);
}

static void _send_off(aoostar_drv_t * drv)
{
    uint8_t buf[8];
    memcpy(buf,      aoostar_sync,     4);
    memcpy(buf + 4,  aoostar_cmd_off,  4);
    write(drv->fd, buf, sizeof(buf));
}

/**********************
 *   STATIC FUNCTIONS — DMA Buffer
 **********************/

/**
 * Build a contiguous DMA-friendly buffer with all dirty chunks.
 *
 * Walks rows in the dirty area, computes touched 48-byte chunks,
 * compares against prev_frame, encodes changed chunks sequentially.
 * Entire buffer can be sent with a single write().
 *
 * @return number of bytes to send
 */
static size_t _build_dma_buf(aoostar_drv_t * drv,
                              const lv_area_t * area,
                              const uint8_t * px_map)
{
    uint8_t * p   = drv->dma_buf;
    uint8_t * end = drv->dma_buf + drv->dma_buf_size;

    /* 1. Frame start marker */
    memcpy(p, aoostar_frame_start, sizeof(aoostar_frame_start));
    p += sizeof(aoostar_frame_start);

    /* 2. Walk dirty rows */
    for(int32_t y = area->y1; y <= area->y2; y++) {
        uint32_t cs = AOOSTAR_ROW_FIRST(y) + AOOSTAR_PIX2CHUNK(area->x1);
        uint32_t ce = AOOSTAR_ROW_LAST(y, area->x2);

        for(uint32_t ci = cs; ci <= ce; ci++) {
            uint32_t off  = ci * AOOSTAR_CHUNK_BYTES;
            uint8_t * cur = (uint8_t *)&px_map[off];

            /* Skip if unchanged from previous frame */
            if(memcmp(cur, &drv->prev_frame[off], AOOSTAR_CHUNK_BYTES) == 0)
                continue;

            /* Ensure room for one chunk packet */
            if(p + AOOSTAR_CHUNK_PACKET_SZ > end) {
                LV_LOG_WARN("DMA buffer overflow, truncating");
                goto done;
            }

            /* Encode: sync(4) + chunk_cmd(4) + offset_LE(4) + data(48) */
            memcpy(p,      aoostar_sync,      4);  p += 4;
            memcpy(p,      aoostar_cmd_chunk, 4);  p += 4;
            *(uint32_t *)p = off;                  p += 4;
            memcpy(p,      cur, AOOSTAR_CHUNK_BYTES);
            p += AOOSTAR_CHUNK_BYTES;

            /* Update cache */
            memcpy(&drv->prev_frame[off], cur, AOOSTAR_CHUNK_BYTES);
        }
    }

done:
    /* 3. Frame end marker */
    memcpy(p, aoostar_frame_end, sizeof(aoostar_frame_end));
    p += sizeof(aoostar_frame_end);

    return (size_t)(p - drv->dma_buf);
}

/**********************
 *   STATIC FUNCTIONS — LVGL callbacks
 **********************/

static void flush_cb(lv_display_t * disp, const lv_area_t * area, uint8_t * px_map)
{
    aoostar_drv_t * drv = lv_display_get_driver_data(disp);
    if(!drv || drv->fd < 0) {
        lv_display_flush_ready(disp);
        return;
    }

    size_t len = _build_dma_buf(drv, area, px_map);

    /* Only send if there's actual data beyond the frame markers */
    if(len > sizeof(aoostar_frame_start) + sizeof(aoostar_frame_end)) {
        write(drv->fd, drv->dma_buf, len);
    }

    lv_display_flush_ready(disp);
}

static void del_event_cb(lv_event_t * e)
{
    if(lv_event_get_code(e) != LV_EVENT_DELETE)
        return;

    lv_display_t * disp = lv_event_get_target(e);
    aoostar_drv_t * drv = lv_display_get_driver_data(disp);
    if(!drv)
        return;

    _send_off(drv);
    _serial_close(drv);

    if(drv->prev_frame) lv_free(drv->prev_frame);
    if(drv->dma_buf)    lv_free(drv->dma_buf);
    lv_free(drv);
    lv_display_set_driver_data(disp, NULL);
}

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

lv_display_t * lv_aoostar_create(void)
{
    aoostar_drv_t * drv = lv_malloc_zeroed(sizeof(aoostar_drv_t));
    if(!drv) { LV_LOG_ERROR("OOM: drv"); return NULL; }
    drv->fd = -1;

    drv->prev_frame = lv_malloc(AOOSTAR_FRAME_BYTES);
    if(!drv->prev_frame) { LV_LOG_ERROR("OOM: prev"); lv_free(drv); return NULL; }
    memset(drv->prev_frame, 0, AOOSTAR_FRAME_BYTES);

    drv->dma_buf_size = AOOSTAR_DMA_BUF_SIZE;
    drv->dma_buf = lv_malloc(drv->dma_buf_size);
    if(!drv->dma_buf) { LV_LOG_ERROR("OOM: dma"); lv_free(drv->prev_frame); lv_free(drv); return NULL; }

    drv->fd = _serial_open("/dev/ttyACM0");

    lv_display_t * disp = lv_display_create(AOOSTAR_HOR_RES, AOOSTAR_VER_RES);
    if(!disp) {
        LV_LOG_ERROR("lv_display_create failed");
        _serial_close(drv);
        lv_free(drv->prev_frame);
        lv_free(drv->dma_buf);
        lv_free(drv);
        return NULL;
    }

    lv_display_set_driver_data(disp, drv);
    lv_display_set_flush_cb(disp, flush_cb);
    lv_display_set_color_format(disp, LV_COLOR_FORMAT_RGB565);
    lv_display_add_event_cb(disp, del_event_cb, LV_EVENT_DELETE, NULL);

    uint32_t buf_size = AOOSTAR_FRAME_BYTES;
    uint8_t * buf1 = lv_malloc(buf_size);
    uint8_t * buf2 = lv_malloc(buf_size);
    if(!buf1 || !buf2) {
        LV_LOG_ERROR("OOM: draw bufs");
        lv_free(buf1); lv_free(buf2);
        lv_display_delete(disp);
        return NULL;
    }
    lv_display_set_buffers(disp, buf1, buf2, buf_size, LV_DISPLAY_RENDER_MODE_DIRECT);

    if(drv->fd >= 0) {
        _send_on(drv);
        lv_area_t full = {0, 0, AOOSTAR_HOR_RES - 1, AOOSTAR_VER_RES - 1};
        lv_memset(buf1, 0, buf_size);
        size_t len = _build_dma_buf(drv, &full, buf1);
        write(drv->fd, drv->dma_buf, len);
    }

    return disp;
}

lv_result_t lv_aoostar_set_device(lv_display_t * disp, const char * device)
{
    aoostar_drv_t * drv = lv_display_get_driver_data(disp);
    if(!drv)
        return LV_RESULT_INVALID;

    if(drv->fd >= 0) {
        _send_off(drv);
        _serial_close(drv);
    }

    drv->fd = _serial_open(device);
    if(drv->fd < 0)
        return LV_RESULT_INVALID;

    _send_on(drv);
    memset(drv->prev_frame, 0, AOOSTAR_FRAME_BYTES);
    LV_LOG_INFO("AOOSTAR switched to %s", device);
    return LV_RESULT_OK;
}

#endif /* LV_USE_AOOSTAR */
