/**
 * @file test_gnss_parser.c
 * @brief Unit tests for NMEA and UBX parsers
 *
 * Build:  gcc -Wall -Wextra -o test_gnss test/test_gnss_parser.c \
 *              src/nmea_parser.c src/ubx_parser.c src/gnss_parser.c \
 *              -Iinclude -lm
 * Run:    ./test_gnss
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "gnss_parser.h"

/* ──────────────────────────────────────────────
   Simple test harness
   ────────────────────────────────────────────── */

static int tests_run    = 0;
static int tests_passed = 0;
static int tests_failed = 0;

#define ASSERT(cond, msg) do {                                      \
    tests_run++;                                                    \
    if (cond) {                                                     \
        tests_passed++;                                             \
        printf("  \033[32m✓\033[0m %s\n", msg);                    \
    } else {                                                        \
        tests_failed++;                                             \
        printf("  \033[31m✗\033[0m %s  (line %d)\n", msg, __LINE__);\
    }                                                               \
} while(0)

#define ASSERT_DBL(a, b, eps, msg) \
    ASSERT(fabs((a)-(b)) < (eps), msg)

#define TEST_GROUP(name) printf("\n── %s ──\n", name)

/* ──────────────────────────────────────────────
   Test data
   ────────────────────────────────────────────── */

/* Valid GGA with fix */
static const char *VALID_GGA =
    "$GPGGA,092750.000,5321.6802,N,00630.3372,W,1,8,1.03,61.7,M,55.2,M,,*76";

/* Valid RMC (active) */
static const char *VALID_RMC =
    "$GPRMC,092750.000,A,5321.6802,N,00630.3372,W,0.02,31.66,280511,,,A*43";

/* Valid GSA */
static const char *VALID_GSA =
    "$GPGSA,A,3,04,05,,09,12,,,24,,,,2.5,1.3,2.1*39";

/* Valid VTG */
static const char *VALID_VTG =
    "$GPVTG,054.7,T,034.4,M,005.5,N,010.2,K,A*27";

/* Bad checksum */
static const char *BAD_CK_GGA =
    "$GPGGA,092750.000,5321.6802,N,00630.3372,W,1,8,1.03,61.7,M,55.2,M,,*FF";

/* UBX-NAV-PVT synthetic frame (92 byte payload) */
static uint8_t make_ubx_nav_pvt_frame(uint8_t *out) {
    /* Header */
    out[0] = 0xB5; out[1] = 0x62;
    out[2] = UBX_CLASS_NAV; out[3] = UBX_ID_NAV_PVT;
    uint16_t plen = sizeof(ubx_nav_pvt_t);
    out[4] = plen & 0xFF; out[5] = (plen >> 8) & 0xFF;

    /* Payload: construct a NAV-PVT */
    ubx_nav_pvt_t pvt;
    memset(&pvt, 0, sizeof(pvt));
    pvt.iTOW    = 450000000;   /* 450000 s */
    pvt.year    = 2024;
    pvt.month   = 6;
    pvt.day     = 15;
    pvt.hour    = 12;
    pvt.min     = 30;
    pvt.sec     = 0;
    pvt.fixType = 3;           /* 3D fix */
    pvt.numSV   = 10;
    pvt.lat     = 533216802;   /* 53.3216802 deg * 1e7 */
    pvt.lon     = -63033720;   /* -6.303372 deg * 1e7  */
    pvt.hMSL    = 61700;       /* 61.7 m in mm         */
    pvt.gSpeed  = 103;         /* 0.103 m/s            */
    pvt.hAcc    = 2500;        /* 2.5 m                */
    pvt.vAcc    = 4000;        /* 4.0 m                */
    memcpy(&out[6], &pvt, sizeof(pvt));

    /* Checksum */
    uint8_t ck_a, ck_b;
    ubx_compute_checksum(&out[2], 4 + plen, &ck_a, &ck_b);
    out[6 + plen]     = ck_a;
    out[6 + plen + 1] = ck_b;

    return (uint8_t)(6 + plen + 2);
}

/* UBX-ACK-ACK */
static uint8_t make_ubx_ack_frame(uint8_t *out, bool is_ack) {
    out[0] = 0xB5; out[1] = 0x62;
    out[2] = UBX_CLASS_ACK;
    out[3] = is_ack ? UBX_ID_ACK_ACK : UBX_ID_ACK_NAK;
    out[4] = 2; out[5] = 0;     /* 2-byte payload */
    out[6] = UBX_CLASS_NAV;     /* clsID: we ack'd NAV-PVT */
    out[7] = UBX_ID_NAV_PVT;

    uint8_t ck_a, ck_b;
    ubx_compute_checksum(&out[2], 6, &ck_a, &ck_b);
    out[8] = ck_a; out[9] = ck_b;
    return 10;
}

/* ──────────────────────────────────────────────
   NMEA tests
   ────────────────────────────────────────────── */

static void test_nmea_gga(void) {
    TEST_GROUP("NMEA GGA");
    nmea_gga_t gga;

    gnss_err_t err = nmea_parse_gga(VALID_GGA, &gga);
    ASSERT(err == GNSS_OK,                    "GGA parse succeeds");
    ASSERT(gga.valid,                          "GGA reports valid fix");
    ASSERT(gga.hour   == 9,                    "Hour = 09");
    ASSERT(gga.minute == 27,                   "Minute = 27");
    ASSERT(gga.second == 50,                   "Second = 50");
    ASSERT(gga.fix_quality == GPS_FIX_GPS,     "Fix quality = GPS");
    ASSERT(gga.num_satellites == 8,            "8 satellites");
    ASSERT_DBL(gga.hdop,        1.03, 0.01,    "HDOP = 1.03");
    ASSERT_DBL(gga.altitude_m,  61.7, 0.1,     "Altitude = 61.7 m");
    ASSERT_DBL(gga.latitude.decimal,  53.3613, 0.001, "Latitude ~53.36°N");
    ASSERT_DBL(gga.longitude.decimal, -6.5056, 0.001, "Longitude ~6.51°W");

    err = nmea_parse_gga(BAD_CK_GGA, &gga);
    ASSERT(err == GNSS_ERR_CHECKSUM,           "Bad checksum detected");

    err = nmea_parse_gga(NULL, &gga);
    ASSERT(err == GNSS_ERR_INVALID,            "NULL sentence rejected");

    err = nmea_parse_gga(VALID_GGA, NULL);
    ASSERT(err == GNSS_ERR_INVALID,            "NULL output rejected");
}

static void test_nmea_rmc(void) {
    TEST_GROUP("NMEA RMC");
    nmea_rmc_t rmc;

    gnss_err_t err = nmea_parse_rmc(VALID_RMC, &rmc);
    ASSERT(err == GNSS_OK,          "RMC parse succeeds");
    ASSERT(rmc.valid,                "Status = A (active)");
    ASSERT(rmc.status == 'A',        "Status char = A");
    ASSERT(rmc.day   == 28,          "Day = 28");
    ASSERT(rmc.month == 5,           "Month = 05");
    ASSERT(rmc.year  == 2011,        "Year = 2011");
    ASSERT_DBL(rmc.speed_knots, 0.02, 0.001, "Speed = 0.02 kn");
    ASSERT_DBL(rmc.course_deg, 31.66, 0.01,  "Course = 31.66°");
}

static void test_nmea_gsa(void) {
    TEST_GROUP("NMEA GSA");
    nmea_gsa_t gsa;

    gnss_err_t err = nmea_parse_gsa(VALID_GSA, &gsa);
    ASSERT(err == GNSS_OK,         "GSA parse succeeds");
    ASSERT(gsa.mode == 'A',         "Auto mode");
    ASSERT(gsa.fix_type == 3,       "3D fix");
    ASSERT_DBL(gsa.pdop, 2.5, 0.01, "PDOP = 2.5");
    ASSERT_DBL(gsa.hdop, 1.3, 0.01, "HDOP = 1.3");
    ASSERT_DBL(gsa.vdop, 2.1, 0.01, "VDOP = 2.1");
}

static void test_nmea_vtg(void) {
    TEST_GROUP("NMEA VTG");
    nmea_vtg_t vtg;

    gnss_err_t err = nmea_parse_vtg(VALID_VTG, &vtg);
    ASSERT(err == GNSS_OK,                     "VTG parse succeeds");
    ASSERT_DBL(vtg.course_true_deg, 54.7, 0.1, "True course = 54.7°");
    ASSERT_DBL(vtg.course_mag_deg,  34.4, 0.1, "Mag course  = 34.4°");
    ASSERT_DBL(vtg.speed_knots,      5.5, 0.01,"Speed = 5.5 kn");
    ASSERT_DBL(vtg.speed_kmh,       10.2, 0.01,"Speed = 10.2 km/h");
}

static void test_nmea_checksum(void) {
    TEST_GROUP("NMEA Checksum");
    /* Known-good: XOR of content between $ and * */
    uint8_t ck = nmea_checksum("$GPGLL,3723.2475,N,12158.3416,W,161229.487,A,A*41");
    ASSERT(ck == 0x41, "Checksum of GPGLL = 0x41");
}

/* ──────────────────────────────────────────────
   UBX tests
   ────────────────────────────────────────────── */

static void test_ubx_nav_pvt(void) {
    TEST_GROUP("UBX NAV-PVT");

    uint8_t frame[300];
    uint8_t frame_len = make_ubx_nav_pvt_frame(frame);

    ASSERT(ubx_verify_checksum(frame, frame_len), "NAV-PVT checksum valid");

    ubx_message_t msg;
    gnss_err_t err = ubx_parse_message(frame, frame_len, &msg);
    ASSERT(err == GNSS_OK,                      "NAV-PVT parse succeeds");
    ASSERT(msg.type == UBX_MSG_NAV_PVT,          "Correct message type");

    ubx_nav_pvt_t *pvt = &msg.payload.nav_pvt;
    ASSERT(pvt->fixType == 3,                    "Fix type = 3D");
    ASSERT(pvt->numSV   == 10,                   "10 satellites");
    ASSERT(pvt->year    == 2024,                 "Year = 2024");
    ASSERT(pvt->month   == 6,                    "Month = 6");
    ASSERT(pvt->day     == 15,                   "Day = 15");
    ASSERT_DBL(pvt->lat / 1e7, 53.3216802, 1e-6,"Latitude correct");
    ASSERT_DBL(pvt->lon / 1e7, -6.303372,  1e-5,"Longitude correct");
    ASSERT(pvt->hMSL == 61700,                   "Height MSL = 61700 mm");
}

static void test_ubx_ack(void) {
    TEST_GROUP("UBX ACK-ACK / ACK-NAK");

    uint8_t frame[10];
    ubx_message_t msg;

    uint8_t len = make_ubx_ack_frame(frame, true);
    gnss_err_t err = ubx_parse_message(frame, len, &msg);
    ASSERT(err == GNSS_OK,             "ACK-ACK parse succeeds");
    ASSERT(msg.type == UBX_MSG_ACK_ACK,"Type = ACK-ACK");
    ASSERT(msg.payload.ack.clsID == UBX_CLASS_NAV, "ACK'd class = NAV");
    ASSERT(msg.payload.ack.msgID == UBX_ID_NAV_PVT, "ACK'd id = PVT");

    len = make_ubx_ack_frame(frame, false);
    err = ubx_parse_message(frame, len, &msg);
    ASSERT(err == GNSS_OK,             "ACK-NAK parse succeeds");
    ASSERT(msg.type == UBX_MSG_ACK_NAK,"Type = ACK-NAK");
}

static void test_ubx_bad_checksum(void) {
    TEST_GROUP("UBX bad checksum");

    uint8_t frame[300];
    uint8_t len = make_ubx_nav_pvt_frame(frame);
    frame[len - 1] ^= 0xFF;   /* corrupt last checksum byte */

    ASSERT(!ubx_verify_checksum(frame, len), "Corrupted frame rejected");

    ubx_message_t msg;
    gnss_err_t err = ubx_parse_message(frame, len, &msg);
    ASSERT(err == GNSS_ERR_CHECKSUM, "parse returns CHECKSUM error");
}

/* ──────────────────────────────────────────────
   Streaming parser tests
   ────────────────────────────────────────────── */

static int   cb_nmea_count = 0;
static int   cb_ubx_count  = 0;
static char  last_nmea[GNSS_NMEA_MAX_LEN];
static ubx_message_t last_ubx;

static void on_nmea(const char *s, void *ctx) {
    (void)ctx;
    cb_nmea_count++;
    strncpy(last_nmea, s, sizeof(last_nmea) - 1);
}

static void on_ubx(const ubx_message_t *m, void *ctx) {
    (void)ctx;
    cb_ubx_count++;
    last_ubx = *m;
}

static void test_streaming_parser(void) {
    TEST_GROUP("Streaming byte-feed parser");

    gnss_parser_ctx_t ctx;
    gnss_parser_init(&ctx, on_nmea, on_ubx, NULL);
    cb_nmea_count = 0;
    cb_ubx_count  = 0;

    /* Feed a complete NMEA sentence followed by CRLF */
    const char *sentence = "$GPGGA,092750.000,5321.6802,N,00630.3372,W,1,8,1.03,61.7,M,55.2,M,,*76\r\n";
    gnss_parser_feed(&ctx, (const uint8_t *)sentence, strlen(sentence));
    ASSERT(cb_nmea_count == 1, "Streaming: NMEA callback fires once");
    ASSERT(last_nmea[0] == '$', "Streaming: sentence starts with $");

    /* Feed two NMEA sentences byte-by-byte */
    const char *s2 = "$GPRMC,092750.000,A,5321.6802,N,00630.3372,W,0.02,31.66,280511,,,A*43\r\n";
    for (size_t j = 0; j < strlen(s2); j++)
        gnss_parser_feed(&ctx, (const uint8_t *)&s2[j], 1);
    ASSERT(cb_nmea_count == 2, "Byte-by-byte: second NMEA fires");

    /* Feed a UBX NAV-PVT frame */
    uint8_t frame[300];
    uint8_t flen = make_ubx_nav_pvt_frame(frame);
    gnss_parser_feed(&ctx, frame, flen);
    ASSERT(cb_ubx_count == 1,                  "UBX callback fires");
    ASSERT(last_ubx.type == UBX_MSG_NAV_PVT,   "UBX type = NAV-PVT");

    /* Mixed stream: NMEA then UBX in one buffer */
    cb_nmea_count = 0; cb_ubx_count = 0;
    uint8_t mixed[600];
    size_t off = 0;
    memcpy(mixed + off, sentence, strlen(sentence)); off += strlen(sentence);
    memcpy(mixed + off, frame, flen);                off += flen;
    gnss_parser_feed(&ctx, mixed, off);
    ASSERT(cb_nmea_count == 1, "Mixed stream: NMEA fires");
    ASSERT(cb_ubx_count  == 1, "Mixed stream: UBX fires");
}

/* ──────────────────────────────────────────────
   Utility tests
   ────────────────────────────────────────────── */

static void test_haversine(void) {
    TEST_GROUP("Haversine distance");

    /* London Heathrow → Amsterdam Schiphol ~360 km */
    double d = gnss_haversine_m(51.4775, -0.4614, 52.3086, 4.7639);
    ASSERT(d > 350000.0 && d < 400000.0, "LHR→AMS ~370 km");

    /* Same point = 0 */
    d = gnss_haversine_m(48.8566, 2.3522, 48.8566, 2.3522);
    ASSERT_DBL(d, 0.0, 0.001, "Same point = 0 m");
}

static void test_fix_type_str(void) {
    TEST_GROUP("Fix type strings");
    ASSERT(strcmp(gnss_fix_type_str(0), "No Fix")       == 0, "0 = No Fix");
    ASSERT(strcmp(gnss_fix_type_str(3), "3D Fix")        == 0, "3 = 3D Fix");
    ASSERT(strcmp(gnss_fix_type_str(5), "Time Only")     == 0, "5 = Time Only");
    ASSERT(strcmp(gnss_fix_type_str(9), "Unknown")       == 0, "9 = Unknown");
}

/* ──────────────────────────────────────────────
   Main
   ────────────────────────────────────────────── */

int main(void) {
    printf("\n╔══════════════════════════════════════════╗\n");
    printf("║  u-blox GNSS Parser — Test Suite          ║\n");
    printf("╚══════════════════════════════════════════╝\n");

    test_nmea_checksum();
    test_nmea_gga();
    test_nmea_rmc();
    test_nmea_gsa();
    test_nmea_vtg();
    test_ubx_nav_pvt();
    test_ubx_ack();
    test_ubx_bad_checksum();
    test_streaming_parser();
    test_haversine();
    test_fix_type_str();

    printf("\n══════════════════════════════════════════\n");
    printf("  Results: %d/%d passed", tests_passed, tests_run);
    if (tests_failed > 0)
        printf("  (\033[31m%d FAILED\033[0m)", tests_failed);
    else
        printf("  \033[32m(all good!)\033[0m");
    printf("\n══════════════════════════════════════════\n\n");

    return tests_failed > 0 ? 1 : 0;
}
