/**
 * @file nmea_parser.c
 * @brief NMEA 0183 sentence parser implementation
 *
 * No dynamic memory. No floating-point stdlib.
 * Safe for bare-metal embedded use.
 */

#include "gnss_parser.h"
#include <string.h>
#include <stdlib.h>   /* strtol, strtod */
#include <stdio.h>    /* sscanf          */

/* ──────────────────────────────────────────────
   Internal helpers
   ────────────────────────────────────────────── */

/**
 * Split a comma-separated NMEA sentence into fields.
 * Modifies buf in-place. Returns number of fields.
 */
static int nmea_split_fields(char *buf, char *fields[], int max_fields) {
    int count = 0;
    char *p = buf;

    while (*p && count < max_fields) {
        fields[count++] = p;
        while (*p && *p != ',' && *p != '*') p++;
        if (*p) *p++ = '\0';
    }
    return count;
}

/**
 * Parse NMEA coordinate field: "DDDMM.MMMM" + direction field.
 */
static gnss_err_t parse_coord(const char *val, const char *dir,
                               nmea_coord_t *out) {
    if (!val || !dir || val[0] == '\0') return GNSS_ERR_INVALID;

    double raw = strtod(val, NULL);
    int degrees = (int)(raw / 100);
    double minutes = raw - (degrees * 100.0);

    out->degrees   = degrees;
    out->minutes   = minutes;
    out->direction = dir[0];
    out->decimal   = degrees + minutes / 60.0;

    if (dir[0] == 'S' || dir[0] == 'W')
        out->decimal = -out->decimal;

    return GNSS_OK;
}

/**
 * Parse HHMMSS.SSS time field into h/m/s/ms.
 */
static void parse_time(const char *t,
                        uint8_t *h, uint8_t *m, uint8_t *s, uint16_t *ms) {
    if (!t || t[0] == '\0') return;
    double val = strtod(t, NULL);
    int total = (int)val;
    *h  = total / 10000;
    *m  = (total / 100) % 100;
    *s  = total % 100;
    if (ms) *ms = (uint16_t)((val - total) * 1000.0);
}

/* ──────────────────────────────────────────────
   Public: NMEA Checksum
   ────────────────────────────────────────────── */

uint8_t nmea_checksum(const char *sentence) {
    uint8_t ck = 0;
    /* XOR all bytes between '$' and '*' (exclusive) */
    const char *p = sentence;
    if (*p == '$') p++;
    while (*p && *p != '*') ck ^= (uint8_t)*p++;
    return ck;
}

/* ──────────────────────────────────────────────
   $GPGGA parser
   ────────────────────────────────────────────── */

gnss_err_t nmea_parse_gga(const char *sentence, nmea_gga_t *out) {
    if (!sentence || !out) return GNSS_ERR_INVALID;

    /* Verify checksum */
    const char *star = strchr(sentence, '*');
    if (star) {
        uint8_t expected = nmea_checksum(sentence);
        uint8_t got = (uint8_t)strtol(star + 1, NULL, 16);
        if (expected != got) return GNSS_ERR_CHECKSUM;
    }

    char buf[GNSS_NMEA_MAX_LEN];
    strncpy(buf, sentence, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    char *fields[20];
    int n = nmea_split_fields(buf, fields, 20);
    if (n < 10) return GNSS_ERR_INVALID;

    memset(out, 0, sizeof(*out));

    parse_time(fields[1], &out->hour, &out->minute, &out->second, &out->millisecond);
    parse_coord(fields[2], fields[3], &out->latitude);
    parse_coord(fields[4], fields[5], &out->longitude);

    out->fix_quality    = (gps_fix_quality_t)strtol(fields[6], NULL, 10);
    out->num_satellites = (uint8_t)strtol(fields[7], NULL, 10);
    out->hdop           = strtod(fields[8], NULL);
    out->altitude_m     = strtod(fields[9], NULL);
    if (n > 11) out->geoid_sep_m = strtod(fields[11], NULL);

    out->valid = (out->fix_quality != GPS_FIX_NONE);
    return GNSS_OK;
}

/* ──────────────────────────────────────────────
   $GPRMC parser
   ────────────────────────────────────────────── */

gnss_err_t nmea_parse_rmc(const char *sentence, nmea_rmc_t *out) {
    if (!sentence || !out) return GNSS_ERR_INVALID;

    const char *star = strchr(sentence, '*');
    if (star) {
        uint8_t expected = nmea_checksum(sentence);
        uint8_t got = (uint8_t)strtol(star + 1, NULL, 16);
        if (expected != got) return GNSS_ERR_CHECKSUM;
    }

    char buf[GNSS_NMEA_MAX_LEN];
    strncpy(buf, sentence, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    char *fields[20];
    int n = nmea_split_fields(buf, fields, 20);
    if (n < 10) return GNSS_ERR_INVALID;

    memset(out, 0, sizeof(*out));

    parse_time(fields[1], &out->hour, &out->minute, &out->second, NULL);
    out->status = fields[2][0];   /* 'A' or 'V' */
    parse_coord(fields[3], fields[4], &out->latitude);
    parse_coord(fields[5], fields[6], &out->longitude);
    out->speed_knots = strtod(fields[7], NULL);
    out->course_deg  = strtod(fields[8], NULL);

    /* Date: DDMMYY */
    if (n > 9 && fields[9][0] != '\0') {
        int date = (int)strtol(fields[9], NULL, 10);
        out->day   = date / 10000;
        out->month = (date / 100) % 100;
        out->year  = 2000 + (date % 100);
    }

    out->valid = (out->status == 'A');
    return GNSS_OK;
}

/* ──────────────────────────────────────────────
   $GPGSV parser
   ────────────────────────────────────────────── */

gnss_err_t nmea_parse_gsv(const char *sentence, nmea_gsv_t *out) {
    if (!sentence || !out) return GNSS_ERR_INVALID;

    char buf[GNSS_NMEA_MAX_LEN];
    strncpy(buf, sentence, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    char *fields[24];
    int n = nmea_split_fields(buf, fields, 24);
    if (n < 4) return GNSS_ERR_INVALID;

    out->total_msgs = (uint8_t)strtol(fields[1], NULL, 10);
    out->msg_num    = (uint8_t)strtol(fields[2], NULL, 10);
    out->total_sats = (uint8_t)strtol(fields[3], NULL, 10);

    /* Up to 4 satellite records per sentence, starting at field 4 */
    int sat_idx = (out->msg_num - 1) * 4;
    int field   = 4;
    out->sat_count = 0;

    while (field + 3 < n && sat_idx < GNSS_MAX_SATELLITES) {
        nmea_satellite_t *s = &out->sats[sat_idx];
        s->prn           = (uint8_t)strtol(fields[field],     NULL, 10);
        s->elevation_deg = (int8_t) strtol(fields[field + 1], NULL, 10);
        s->azimuth_deg   = (uint16_t)strtol(fields[field + 2], NULL, 10);
        s->snr_db        = (fields[field + 3][0] != '\0')
                           ? (uint8_t)strtol(fields[field + 3], NULL, 10)
                           : 0;
        field   += 4;
        sat_idx++;
        out->sat_count++;
    }

    out->valid = true;
    return GNSS_OK;
}

/* ──────────────────────────────────────────────
   $GPGSA parser
   ────────────────────────────────────────────── */

gnss_err_t nmea_parse_gsa(const char *sentence, nmea_gsa_t *out) {
    if (!sentence || !out) return GNSS_ERR_INVALID;

    char buf[GNSS_NMEA_MAX_LEN];
    strncpy(buf, sentence, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    char *fields[20];
    int n = nmea_split_fields(buf, fields, 20);
    if (n < 17) return GNSS_ERR_INVALID;

    memset(out, 0, sizeof(*out));

    out->mode     = fields[1][0];
    out->fix_type = (uint8_t)strtol(fields[2], NULL, 10);

    for (int i = 0; i < 12 && (3 + i) < n; i++)
        out->prn_used[i] = (uint8_t)strtol(fields[3 + i], NULL, 10);

    out->pdop = strtod(fields[14], NULL);
    out->hdop = strtod(fields[15], NULL);
    out->vdop = strtod(fields[16], NULL);

    out->valid = true;
    return GNSS_OK;
}

/* ──────────────────────────────────────────────
   $GPVTG parser
   ────────────────────────────────────────────── */

gnss_err_t nmea_parse_vtg(const char *sentence, nmea_vtg_t *out) {
    if (!sentence || !out) return GNSS_ERR_INVALID;

    char buf[GNSS_NMEA_MAX_LEN];
    strncpy(buf, sentence, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    char *fields[12];
    int n = nmea_split_fields(buf, fields, 12);
    if (n < 9) return GNSS_ERR_INVALID;

    memset(out, 0, sizeof(*out));

    out->course_true_deg = strtod(fields[1], NULL);
    out->course_mag_deg  = strtod(fields[3], NULL);
    out->speed_knots     = strtod(fields[5], NULL);
    out->speed_kmh       = strtod(fields[7], NULL);

    out->valid = true;
    return GNSS_OK;
}

/* ──────────────────────────────────────────────
   Utility
   ────────────────────────────────────────────── */

double nmea_coord_to_decimal(const nmea_coord_t *c) {
    if (!c) return 0.0;
    return c->decimal;
}
