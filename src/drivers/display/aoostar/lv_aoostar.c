/**
 * @file lv_aoostar.c
 *
 * AOOSTAR GEM12 / WTR MAX secondary screen display driver.
 * 960×376 RGB565 over USB CDC ACM @ 1.5M baud.
 *
 * Protocol: AA 55 AA 55 [cmd] [offset LE] [data]
 * Chunk size: 48 bytes, per-chunk write_all (no DMA).
 */

#include "lv_aoostar.h"

#if LV_USE_AOOSTAR

#ifndef _DEFAULT_SOURCE
#define _DEFAULT_SOURCE
#endif
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdint.h>
#include <errno.h>
#include <termios.h>

#include "../../../display/lv_display_private.h"
#include "../../../misc/lv_types.h"
#include "../../../misc/lv_log.h"

#define AOOSTAR_HOR_RES         960
#define AOOSTAR_VER_RES         376
#define AOOSTAR_BPP             2
#define AOOSTAR_FRAME_BYTES     (AOOSTAR_HOR_RES * AOOSTAR_VER_RES * AOOSTAR_BPP)
#define AOOSTAR_CHUNK_BYTES     48
#define AOOSTAR_CHUNKS_PER_ROW  (AOOSTAR_HOR_RES * AOOSTAR_BPP / AOOSTAR_CHUNK_BYTES)
#define AOOSTAR_TOTAL_CHUNKS    (AOOSTAR_VER_RES * AOOSTAR_CHUNKS_PER_ROW)
#define AOOSTAR_CHUNK_PACKET_SZ (4 + 4 + 4 + AOOSTAR_CHUNK_BYTES)

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

static void write_all(int fd, const void *data, size_t len) {
    const uint8_t *p = (const uint8_t *)data;
    while (len > 0) {
        ssize_t n = write(fd, p, len);
        if (n < 0) {
            if (errno == EINTR || errno == EAGAIN) continue;
            LV_LOG_ERROR("serial write: %s", strerror(errno));
            return;
        }
        p += n;
        len -= n;
    }
}

typedef struct {
    int       fd;
    uint8_t * prev_frame;
} aoostar_drv_t;

static int  _serial_open(const char * device);
static void _serial_close(aoostar_drv_t * drv);
static void _send_on(aoostar_drv_t * drv);
static void _send_off(aoostar_drv_t * drv);
static void flush_cb(lv_display_t * disp, const lv_area_t * area, uint8_t * px_map);
static void del_event_cb(lv_event_t * e);

static int _serial_open(const char * device)
{
    int fd = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        LV_LOG_ERROR("Failed to open %s: %s", device, strerror(errno));
        return -1;
    }

    struct termios tio;
    memset(&tio, 0, sizeof(tio));
    if (tcgetattr(fd, &tio) < 0) {
        LV_LOG_ERROR("tcgetattr: %s", strerror(errno));
        close(fd);
        return -1;
    }

    cfmakeraw(&tio);
    tio.c_cflag |= CLOCAL | CREAD;
    tio.c_cc[VMIN]  = 0;
    tio.c_cc[VTIME] = 10;

    if (cfsetspeed(&tio, B1500000) < 0) {
        LV_LOG_ERROR("cfsetspeed B1500000: %s", strerror(errno));
        close(fd);
        return -1;
    }

    if (tcsetattr(fd, TCSANOW, &tio) < 0) {
        LV_LOG_ERROR("tcsetattr: %s", strerror(errno));
        close(fd);
        return -1;
    }

    tcflush(fd, TCIOFLUSH);
    LV_LOG_INFO("Serial %s: B1500000 8N1", device);
    return fd;
}

static void _serial_close(aoostar_drv_t * drv)
{
    if (drv->fd >= 0) {
        close(drv->fd);
        drv->fd = -1;
    }
}

static void _send_on(aoostar_drv_t * drv)
{
    uint8_t buf[8];
    memcpy(buf,      aoostar_sync,    4);
    memcpy(buf + 4,  aoostar_cmd_on,  4);
    write_all(drv->fd, buf, 8);
    usleep(500000);
    tcflush(drv->fd, TCIFLUSH);
}

static void _send_off(aoostar_drv_t * drv)
{
    uint8_t buf[8];
    memcpy(buf,      aoostar_sync,     4);
    memcpy(buf + 4,  aoostar_cmd_off,  4);
    write_all(drv->fd, buf, 8);
}

static void flush_cb(lv_display_t * disp, const lv_area_t * area, uint8_t * px_map)
{
    aoostar_drv_t * drv = lv_display_get_driver_data(disp);
    if (!drv || drv->fd < 0) {
        lv_display_flush_ready(disp);
        return;
    }

    uint8_t pkt[AOOSTAR_CHUNK_PACKET_SZ];

    write_all(drv->fd, aoostar_frame_start, sizeof(aoostar_frame_start));

    for (int32_t y = area->y1; y <= area->y2; y++) {
        uint32_t cs = (uint32_t)y * AOOSTAR_CHUNKS_PER_ROW + (area->x1 / 24);
        uint32_t ce = (uint32_t)y * AOOSTAR_CHUNKS_PER_ROW + ((area->x2 * 2 + 1) / AOOSTAR_CHUNK_BYTES);

        for (uint32_t ci = cs; ci <= ce; ci++) {
            uint32_t off = ci * AOOSTAR_CHUNK_BYTES;
            uint8_t * cur = (uint8_t *)&px_map[off];

            if (memcmp(cur, &drv->prev_frame[off], AOOSTAR_CHUNK_BYTES) == 0)
                continue;

            memcpy(pkt,         aoostar_sync,      4);
            memcpy(pkt + 4,     aoostar_cmd_chunk, 4);
            *(uint32_t *)(pkt + 8) = off;
            memcpy(pkt + 12,    cur, AOOSTAR_CHUNK_BYTES);

            write_all(drv->fd, pkt, AOOSTAR_CHUNK_PACKET_SZ);
            memcpy(&drv->prev_frame[off], cur, AOOSTAR_CHUNK_BYTES);
        }
    }

    write_all(drv->fd, aoostar_frame_end, sizeof(aoostar_frame_end));
    lv_display_flush_ready(disp);
}

static void del_event_cb(lv_event_t * e)
{
    if (lv_event_get_code(e) != LV_EVENT_DELETE)
        return;

    lv_display_t * disp = lv_event_get_target(e);
    aoostar_drv_t * drv = lv_display_get_driver_data(disp);
    if (!drv) return;

    _send_off(drv);
    _serial_close(drv);
    if (drv->prev_frame) lv_free(drv->prev_frame);
    lv_free(drv);
    lv_display_set_driver_data(disp, NULL);
}

lv_display_t * lv_aoostar_create(void)
{
    aoostar_drv_t * drv = lv_malloc_zeroed(sizeof(aoostar_drv_t));
    if (!drv) { LV_LOG_ERROR("OOM: drv"); return NULL; }
    drv->fd = -1;

    drv->prev_frame = lv_malloc(AOOSTAR_FRAME_BYTES);
    if (!drv->prev_frame) { LV_LOG_ERROR("OOM: prev_frame"); lv_free(drv); return NULL; }
    memset(drv->prev_frame, 0, AOOSTAR_FRAME_BYTES);

    drv->fd = _serial_open("/dev/ttyACM0");

    lv_display_t * disp = lv_display_create(AOOSTAR_HOR_RES, AOOSTAR_VER_RES);
    if (!disp) {
        LV_LOG_ERROR("lv_display_create failed");
        _serial_close(drv);
        lv_free(drv->prev_frame);
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
    if (!buf1 || !buf2) {
        LV_LOG_ERROR("OOM: draw buffers");
        lv_free(buf1); lv_free(buf2);
        lv_display_delete(disp);
        return NULL;
    }
    lv_display_set_buffers(disp, buf1, buf2, buf_size, LV_DISPLAY_RENDER_MODE_DIRECT);

    LV_LOG_INFO("AOOSTAR %dx%d RGB565 @1.5M", AOOSTAR_HOR_RES, AOOSTAR_VER_RES);

    if (drv->fd >= 0) {
        _send_on(drv);
        lv_area_t full = {0, 0, AOOSTAR_HOR_RES - 1, AOOSTAR_VER_RES - 1};
        flush_cb(disp, &full, buf1);
    }

    return disp;
}

lv_result_t lv_aoostar_set_device(lv_display_t * disp, const char * device)
{
    aoostar_drv_t * drv = lv_display_get_driver_data(disp);
    if (!drv) return LV_RESULT_INVALID;

    if (drv->fd >= 0) { _send_off(drv); _serial_close(drv); }
    drv->fd = _serial_open(device);
    if (drv->fd < 0) return LV_RESULT_INVALID;

    _send_on(drv);
    memset(drv->prev_frame, 0, AOOSTAR_FRAME_BYTES);
    LV_LOG_INFO("AOOSTAR switched to %s", device);
    return LV_RESULT_OK;
}

#endif /* LV_USE_AOOSTAR */
