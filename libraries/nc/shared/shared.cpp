#include "shared.h"

struct FromMarker from_marker = {0, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, 0};
struct FromGPS from_gps = {{0,0,0},{0,0,0},{0,0,0},{0,0,0},0};
struct FromLSM from_lsm = {{0,0,0},0};

struct ToFlightCtrl to_fc = {NAV_COMMS_VERSION, 2, 7, {0, 0, 0}, {0, 0, 0}, 1, 0, {0, 0, -0.8}, 0.5, 0, 0.3};
struct FromFlightCtrl from_fc = {0, 0, 0, {0, 0, 0}, {0, 0, 0}, {0, 0, 0, 0}, 0};
struct ForDebug for_debug = {{0,0,0,0}, {0,0,0}, {0,0,0}};
struct ToDronePort to_dp = {0, 0, 0, 0, {0, 0, 0}, {0, 0, 0}, {0, 0, 0, 0}};

uint8_t marker_flag = 0;
uint8_t gps_pos_flag = 0;
uint8_t gps_vel_flag = 0;
uint8_t lsm_flag = 0;
uint8_t dp_id = 0;

enum NavMode nav_mode_ = NAV_MODE_OFF;
uint8_t nav_mode_request_from_dp = NAV_MODE_OFF;