#include "sensor_bringup.h"
#include <stdio.h>
extern "C"
{
  #include "../uld-driver/inc/vl53l7cx_api.h"

}

BringupReturn sensor_bringup(VL53L7CX_Configuration& Dev, uint16_t sensor_address, char i2c_bus[]) {
    
    // TODO: Update this method to be bool and return false if setup failure
    // uint16_t i2c_address = 0x50; // Correct address is 0x52
    uint8_t isAlive;
    int status;
    // /* Initialize channel com */
    status = vl53l7cx_comms_init(&Dev.platform, sensor_address, i2c_bus);
    if(status) {
        return {"VL53L7CX comms init failed", status};
    } 
    // else {
    //     printf("VL53L7CX comms success\n");
    // }

    // Check if sensor connected
    status = vl53l7cx_is_alive(&Dev, &isAlive);
    if(!isAlive || status)
    {
        return {"VL53L7CX not detected", status};
    }

        // Initialize sensor
    status = vl53l7cx_init(&Dev);
    if(status) {
        return {"VL53L7CX ULD Loading failed", status};
    }

    // Change resolution
    status = vl53l7cx_set_resolution(&Dev, VL53L7CX_RESOLUTION_8X8);

    if(status) {
        return {"VL53L7CX failed to set to 8x8", status};
    }

    uint8_t res = 64u;
    // Get the current resolution
    status = vl53l7cx_get_resolution(&Dev, &res);
    if(status) {
        return {"Resolution not 8x8", status};
    }

    // Wakeup sensor
    // Note this may fail (if sensor was previously not shut down), but everything may still work
    status = vl53l7cx_set_power_mode(&Dev, VL53L7CX_POWER_MODE_WAKEUP);
    if(status) {
        return {"VL53L7CX wakeup failed", status};
    }


    // printf("VL53L7CX ULD ready ! (Version : %s)\n",
    //         VL53L7CX_API_REVISION);
    // Set the ranging mode to continuous 
    status = vl53l7cx_set_ranging_mode(&Dev, VL53L7CX_RANGING_MODE_CONTINUOUS);
    // Set the ranging frequency to 15 Hz (for 8x8 zone can be 1-15, for 4x4 1-60)
    vl53l7cx_set_ranging_frequency_hz(&Dev, 10);

    status = vl53l7cx_start_ranging(&Dev);
    return {"No errors!", status};
}