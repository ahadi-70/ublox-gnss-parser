/**
 * @file ubx_parser.c
 * @brief u-blox UBX binary protocol parser
 *
 * UBX frame structure:
 *   [0xB5][0x62][Class][ID][Len_L][Len_H][Payload...][CK_A][CK_B]
 *
 * Checksum is Fletcher-8 over bytes Class..Payload (inclusive).
 */

#include "gnss_parser.h"
#include <string.h>

/* ──────────────────────────────────────────────
   Checksum
   ────────────────────────────────────────────── */

void ubx_compute_checksum(const uint8_t *buf, size_t payload_len,
                           uint8_t *ck_a, uint8_t *ck_b) {
    /*
     * buf must point to the Class byte (index 2 in a full frame).
     * payload_len = 4 (cls+id+len_lo+len_hi) + actual payload bytes.
     */
    *ck_a = 0;
    *ck_b = 0;
    for (size_t i = 0; i < payload_len; i++) {
        *ck_a += buf[i];
        *ck_b += *ck_a;
    }
}

bool ubx_verify_checksum(const uint8_t *frame, size_t total_len) {
    if (total_len < 8) return false;  /* min: 2 sync + 4 hdr + 2 ck */

    uint16_t payload_len = (uint16_t)frame[4] | ((uint16_t)frame[5] << 8);
    size_t expected_total = 2u + 4u + payload_len + 2u;
    if (total_len < expected_total) return false;

    uint8_t ck_a, ck_b;
    /* Checksum covers Class, ID, Len_L, Len_H + Payload */
    ubx_compute_checksum(&frame[2], 4u + payload_len, &ck_a, &ck_b);

    return (frame[6 + payload_len]     == ck_a) &&
           (frame[6 + payload_len + 1] == ck_b);
}

/* ──────────────────────────────────────────────
   Frame parser
   ────────────────────────────────────────────── */

gnss_err_t ubx_parse_message(const uint8_t *buf, size_t len,
                              ubx_message_t *out) {
    if (!buf || !out || len < 8) return GNSS_ERR_INVALID;
    if (buf[0] != UBX_SYNC1 || buf[1] != UBX_SYNC2) return GNSS_ERR_INVALID;

    if (!ubx_verify_checksum(buf, len)) return GNSS_ERR_CHECKSUM;

    uint8_t  cls         = buf[2];
    uint8_t  id          = buf[3];
    uint16_t payload_len = (uint16_t)buf[4] | ((uint16_t)buf[5] << 8);
    const uint8_t *payload = &buf[6];

    memset(out, 0, sizeof(*out));
    out->type = UBX_MSG_UNKNOWN;

    if (cls == UBX_CLASS_NAV && id == UBX_ID_NAV_PVT) {
        if (payload_len < sizeof(ubx_nav_pvt_t)) return GNSS_ERR_INVALID;
        memcpy(&out->payload.nav_pvt, payload, sizeof(ubx_nav_pvt_t));
        out->type = UBX_MSG_NAV_PVT;

    } else if (cls == UBX_CLASS_NAV && id == UBX_ID_NAV_STATUS) {
        if (payload_len < sizeof(ubx_nav_status_t)) return GNSS_ERR_INVALID;
        memcpy(&out->payload.nav_status, payload, sizeof(ubx_nav_status_t));
        out->type = UBX_MSG_NAV_STATUS;

    } else if (cls == UBX_CLASS_ACK && id == UBX_ID_ACK_ACK) {
        if (payload_len < sizeof(ubx_ack_t)) return GNSS_ERR_INVALID;
        memcpy(&out->payload.ack, payload, sizeof(ubx_ack_t));
        out->type = UBX_MSG_ACK_ACK;

    } else if (cls == UBX_CLASS_ACK && id == UBX_ID_ACK_NAK) {
        if (payload_len < sizeof(ubx_ack_t)) return GNSS_ERR_INVALID;
        memcpy(&out->payload.ack, payload, sizeof(ubx_ack_t));
        out->type = UBX_MSG_ACK_NAK;

    } else {
        return GNSS_ERR_UNKNOWN_MSG;
    }

    return GNSS_OK;
}

/* ──────────────────────────────────────────────
   Utility helpers
   ────────────────────────────────────────────── */

const char *gnss_fix_type_str(uint8_t fix_type) {
    switch (fix_type) {
        case 0: return "No Fix";
        case 1: return "Dead Reckoning";
        case 2: return "2D Fix";
        case 3: return "3D Fix";
        case 4: return "GNSS + Dead Reckoning";
        case 5: return "Time Only";
        default: return "Unknown";
    }
}
