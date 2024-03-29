#ifndef POINT_CALCS_H
#define POINT_CALCS_H
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "hand_msgs/msg/tof64.hpp"
extern "C" {
  #include "../uld-driver/inc/vl53l7cx_api.h"
}

#include <algorithm>
#include <vector>

class PointCalcs {
    private:
        double sin_of_pitch[64];
        double cos_of_pitch[64];
        double sin_of_yaw[64];
        double cos_of_yaw[64];
        const double VL53L5_Zone_Pitch8x8[64] = {
            52.00,56.00,61.00,64.00,64.00,61.00,56.00,52.00,
            56.00,62.00,67.00,70.00,70.00,67.00,62.00,56.00,
            61.00,67.00,76.00,78.00,78.00,76.00,67.00,61.00,
            64.00,70.00,78.00,84.00,84.00,78.00,70.00,64.00,
            64.00,70.00,78.00,84.00,84.00,78.00,70.00,64.00,
            61.00,67.00,76.00,78.00,78.00,76.00,67.00,61.00,
            56.00,62.00,67.00,70.00,70.00,67.00,62.00,56.00,
            52.00,56.00,61.00,64.00,64.00,61.00,56.00,52.00};
        const double VL53L5_Zone_Yaw8x8[64] = {
            135.00,125.40,113.20, 98.13, 81.87, 66.80, 54.60, 45.00,
            144.60,135.00,120.96,101.31, 78.69, 59.04, 45.00, 35.40,
            156.80,149.04,135.00,108.45, 71.55, 45.00, 30.96, 23.20,
            171.87,168.69,161.55,135.00, 45.00, 18.45, 11.31,  8.13,
            188.13,191.31,198.45,225.00,315.00,341.55,348.69,351.87,
            203.20,210.96,225.00,251.55,288.45,315.00,329.04,336.80,
            203.20,225.00,239.04,258.69,281.31,300.96,315.00,324.60,
            225.00,234.60,246.80,261.87,278.13,293.20,305.40,315.00};

        void setup_angle_table();
        
    public:
        PointCalcs(); 
        uint8_t ConvertDist2XYZCoords8x8(VL53L7CX_ResultsData *ResultsData);//, XYZ_ZoneCoordinates_t *XYZ_ZoneCoordinates);
        int what = 13;
        sensor_msgs::msg::PointCloud2 test();
        sensor_msgs::msg::PointCloud2 test_process(hand_msgs::msg::Tof64 tof_in, char side);
        int get_valid_index(hand_msgs::msg::Tof64& tof_in, int zone_number);
};

#endif