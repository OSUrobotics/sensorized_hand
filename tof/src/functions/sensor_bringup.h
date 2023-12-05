#ifndef SENSOR_BRINGUP_H
#define SENSOR_BRINGUP_H
extern "C" {
  #include "../uld-driver/inc/vl53l7cx_api.h"
}

int sensor_bringup(VL53L7CX_Configuration& Dev, uint16_t sensor_address);
#endif