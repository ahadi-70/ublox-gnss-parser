/**
 * @file gnss_parser.c
 * @brief Streaming byte-feed parser & utility functions
 *
 * Designed to be called from a UART ISR or DMA half/complete callback.
 * Re-entrant per context instance.  No globals.
 */

#include "gnss_parser.h"
#include <string.h>
#include <math.h>   /* sqrt, sin, cos, atan2 */

/* ──────────────────────────────────────────────
   Init
   ────────────────────────────────────────────── */

gnss_err_t gnss_parser_init(gnss_parser_ctx_t *ctx,
                              nmea_sentence_cb on_nmea,
                              ubx_message_cb   on_ubx,
                              void            *cb_ctx) {
    if (!ctx) return GNSS_ERR_INVALID;
    memset(ctx, 0, sizeof(*ctx));
    ctx->state   = PARSE_STATE_IDLE;
    ctx->on_nmea = on_nmea;
    ctx->on_ubx  = on_ubx;
    ctx->cb_ctx  = cb_ctx;
    return GNSS_OK;
}

/* ──────────────────────────────────────────────
   Internal: flush completed NMEA sentence
   ────────────────────────────────────────────── */

static void flush_nmea(gnss_parser_ctx_t *ctx) {
    ctx->nmea_buf[ctx->nmea_len] = '\0';
    if (ctx->on_nmea)
        ctx->on_nmea(ctx->nmea_buf, ctx->cb_ctx);
    ctx->nmea_len = 0;
    ctx->state    = PARSE_STATE_IDLE;
}

/* ──────────────────────────────────────────────
   Internal: flush completed UBX frame
   ────────────────────────────────────────────── */

static void flush_ubx(gnss_parser_ctx_t *ctx) {
    ubx_message_t msg;
    gnss_err_t err = ubx_parse_message(ctx->ubx_buf, ctx->ubx_len, &msg);
    if (err == GNSS_OK && ctx->on_ubx)
        ctx->on_ubx(&msg, ctx->cb_ctx);
    ctx->ubx_len = 0;
    ctx->state   = PARSE_STATE_IDLE;
}

/* ──────────────────────────────────────────────
   Streaming feed — the core state machine
   ────────────────────────────────────────────── */

gnss_err_t gnss_parser_feed(gnss_parser_ctx_t *ctx,
                              const uint8_t *buf, size_t len) {
    if (!ctx || !buf) return GNSS_ERR_INVALID;

    for (size_t i = 0; i < len; i++) {
        uint8_t b = buf[i];

        switch (ctx->state) {

        case PARSE_STATE_IDLE:
            if (b == NMEA_START_CHAR) {
                ctx->nmea_len   = 0;
                ctx->nmea_buf[ctx->nmea_len++] = (char)b;
                ctx->state      = PARSE_STATE_NMEA;
            } else if (b == UBX_SYNC1) {
                ctx->ubx_len     = 0;
                ctx->ubx_buf[ctx->ubx_len++] = b;
                ctx->state       = PARSE_STATE_UBX_SYNC2;
            }
            break;

        /* ── NMEA accumulation ─────────────────────────── */
        case PARSE_STATE_NMEA:
            if (b == '\n' || b == '\r') {
                flush_nmea(ctx);
            } else {
                if (ctx->nmea_len < GNSS_NMEA_MAX_LEN - 1) {
                    ctx->nmea_buf[ctx->nmea_len++] = (char)b;
                } else {
                    /* Overflow: discard and reset */
                    ctx->nmea_len = 0;
                    ctx->state    = PARSE_STATE_IDLE;
                }
            }
            break;

        /* ── UBX: waiting for second sync byte ─────────── */
        case PARSE_STATE_UBX_SYNC2:
            if (b == UBX_SYNC2) {
                ctx->ubx_buf[ctx->ubx_len++] = b;
                ctx->state = PARSE_STATE_UBX_HEADER;
            } else {
                ctx->ubx_len = 0;
                ctx->state   = PARSE_STATE_IDLE;
                /* Could be start of NMEA */
                if (b == NMEA_START_CHAR) {
                    ctx->nmea_buf[ctx->nmea_len++] = (char)b;
                    ctx->state = PARSE_STATE_NMEA;
                }
            }
            break;

        /* ── UBX: header (class, id, len_lo, len_hi) ───── */
        case PARSE_STATE_UBX_HEADER:
            ctx->ubx_buf[ctx->ubx_len++] = b;
            if (ctx->ubx_len == 6) {
                /* Bytes 4-5 are payload length (little-endian) */
                ctx->ubx_expected = (uint16_t)ctx->ubx_buf[4]
                                  | ((uint16_t)ctx->ubx_buf[5] << 8);
                if (ctx->ubx_expected > GNSS_UBX_MAX_PAYLOAD) {
                    /* Reject oversized frame */
                    ctx->ubx_len = 0;
                    ctx->state   = PARSE_STATE_IDLE;
                } else {
                    ctx->ubx_ck_a = 0;
                    ctx->ubx_ck_b = 0;
                    ctx->state    = (ctx->ubx_expected > 0)
                                    ? PARSE_STATE_UBX_PAYLOAD
                                    : PARSE_STATE_UBX_CK;
                }
            }
            break;

        /* ── UBX: payload bytes ─────────────────────────── */
        case PARSE_STATE_UBX_PAYLOAD:
            ctx->ubx_buf[ctx->ubx_len++] = b;
            if (ctx->ubx_len == 6u + ctx->ubx_expected)
                ctx->state = PARSE_STATE_UBX_CK;
            break;

        /* ── UBX: two checksum bytes ────────────────────── */
        case PARSE_STATE_UBX_CK:
            ctx->ubx_buf[ctx->ubx_len++] = b;
            if (ctx->ubx_len == 6u + ctx->ubx_expected + 2u)
                flush_ubx(ctx);
            break;

        default:
            ctx->state = PARSE_STATE_IDLE;
            break;
        }
    }
    return GNSS_OK;
}

/* ──────────────────────────────────────────────
   Haversine distance
   ────────────────────────────────────────────── */

#define DEG_TO_RAD(d) ((d) * 3.14159265358979323846 / 180.0)
#define EARTH_RADIUS_M 6371000.0

double gnss_haversine_m(double lat1, double lon1,
                         double lat2, double lon2) {
    double dlat = DEG_TO_RAD(lat2 - lat1);
    double dlon = DEG_TO_RAD(lon2 - lon1);
    double a = sin(dlat / 2.0) * sin(dlat / 2.0)
             + cos(DEG_TO_RAD(lat1)) * cos(DEG_TO_RAD(lat2))
             * sin(dlon / 2.0) * sin(dlon / 2.0);
    return EARTH_RADIUS_M * 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
}
