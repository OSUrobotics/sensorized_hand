// Modified from https://community.st.com/t5/imaging-sensors/vl53l5cx-multi-zone-sensor-get-x-y-z-of-points-relative-to/m-p/172929

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "PointCalcs.h"
#include <stdio.h>
#include "math.h"
#include "cmath" 
extern "C"
{
  #include "../uld-driver/inc/vl53l7cx_api.h"

}
PointCalcs::PointCalcs() {
    setup_angle_table();
}

void PointCalcs::setup_angle_table() {
    uint8_t ZoneNum;
    for (ZoneNum = 0; ZoneNum < 64; ZoneNum++)
    {
        sin_of_pitch[ZoneNum] = sin((VL53L5_Zone_Pitch8x8[ZoneNum])*M_PI/180.0);
        cos_of_pitch[ZoneNum] = cos((VL53L5_Zone_Pitch8x8[ZoneNum])*M_PI/180.0);
        sin_of_yaw[ZoneNum] = sin(VL53L5_Zone_Yaw8x8[ZoneNum]*M_PI/180.0);
        cos_of_yaw[ZoneNum] = cos(VL53L5_Zone_Yaw8x8[ZoneNum]*M_PI/180.0);
    }
}

/* USER CODE END Includes */
 
/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
 
/* USER CODE END PTD */
 
/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 
/* USER CODE END PD */
 
/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
 
/* USER CODE END PM */
 
/* Private variables ---------------------------------------------------------*/
 
// /* USER CODE BEGIN PV */
// int status;
// volatile int IntCount;
// uint8_t p_data_ready;
// VL53L5CX_Configuration 	Dev;
// VL53L5CX_ResultsData 	Results;
// uint8_t resolution, isAlive;
// uint16_t idx;
 
// const double VL53L5_Zone_Pitch8x8[64] = {
// 		52.00,56.00,61.00,64.00,64.00,61.00,56.00,52.00,
// 		56.00,62.00,67.00,70.00,70.00,67.00,62.00,56.00,
// 		61.00,67.00,76.00,78.00,78.00,76.00,67.00,61.00,
// 		64.00,70.00,78.00,84.00,84.00,78.00,70.00,64.00,
// 		64.00,70.00,78.00,84.00,84.00,78.00,70.00,64.00,
// 		61.00,67.00,76.00,78.00,78.00,76.00,67.00,61.00,
// 		56.00,62.00,67.00,70.00,70.00,67.00,62.00,56.00,
// 		52.00,56.00,61.00,64.00,64.00,61.00,56.00,52.00
// };
 
// const double VL53L5_Zone_Yaw8x8[64] = {
// 		135.00,125.40,113.20, 98.13, 81.87, 66.80, 54.60, 45.00,
// 		144.60,135.00,120.96,101.31, 78.69, 59.04, 45.00, 35.40,
// 		156.80,149.04,135.00,108.45, 71.55, 45.00, 30.96, 23.20,
// 		171.87,168.69,161.55,135.00, 45.00, 18.45, 11.31,  8.13,
// 		188.13,191.31,198.45,225.00,315.00,341.55,348.69,351.87,
// 		203.20,210.96,225.00,251.55,288.45,315.00,329.04,336.80,
// 		203.20,225.00,239.04,258.69,281.31,300.96,315.00,324.60,
// 		225.00,234.60,246.80,261.87,278.13,293.20,305.40,315.00
// };
 
 
// PlaneEquation_t PlaneEquation;
// XYZ_ZoneCoordinates_t XYZ_ZoneCoordinates;
 
// double SinOfPitch[64], CosOfPitch[64], SinOfYaw[64], CosOfYaw[64];
// /* USER CODE END PV */
 
// /* Private function prototypes -----------------------------------------------*/
// /* USER CODE BEGIN PFP */
 
 
 
// uint8_t ComputeSinCosTables(void)
// {
// 	//This function will save the math processing time of the code.  If the user wishes to not
// 	//perform this function, these tables can be generated and saved as a constant.
// 	uint8_t ZoneNum;
// 	for (ZoneNum = 0; ZoneNum < 64; ZoneNum++)
// 	{
// 		SinOfPitch[ZoneNum] = sin((VL53L5_Zone_Pitch8x8[ZoneNum])*Pi/180);
// 		CosOfPitch[ZoneNum] = cos((VL53L5_Zone_Pitch8x8[ZoneNum])*Pi/180);
// 		SinOfYaw[ZoneNum] = sin(VL53L5_Zone_Yaw8x8[ZoneNum]*Pi/180);
// 		CosOfYaw[ZoneNum] = cos(VL53L5_Zone_Yaw8x8[ZoneNum]*Pi/180);
// 	}
 
// 	return 0;
// }
 
// uint8_t ConvertDist2XYZCoords8x8(VL53L5CX_ResultsData *ResultsData, XYZ_ZoneCoordinates_t *XYZ_ZoneCoordinates)
// {
// 	uint8_t ZoneNum;
// 	float Hyp;
// 	for (ZoneNum = 0; ZoneNum < 64; ZoneNum++)
// 	{
// 		if ((ResultsData->nb_target_detected[ZoneNum] > 0) && (ResultsData->distance_mm[ZoneNum] > 0) && ((ResultsData->target_status[ZoneNum] == 5) || (ResultsData->target_status[ZoneNum] == 6) || (ResultsData->target_status[ZoneNum] == 9)) )
// 		{
// 			Hyp = ResultsData->distance_mm[ZoneNum]/SinOfPitch[ZoneNum];
// 			XYZ_ZoneCoordinates->Xpos[ZoneNum] = CosOfYaw[ZoneNum]*CosOfPitch[ZoneNum]*Hyp;
// 			XYZ_ZoneCoordinates->Ypos[ZoneNum] = SinOfYaw[ZoneNum]*CosOfPitch[ZoneNum]*Hyp;
// 			XYZ_ZoneCoordinates->Zpos[ZoneNum] = ResultsData->distance_mm[ZoneNum];
// 		}
// 		else
// 		{
// 			XYZ_ZoneCoordinates->Xpos[ZoneNum] = 0;
// 			XYZ_ZoneCoordinates->Ypos[ZoneNum] = 0;
// 			XYZ_ZoneCoordinates->Zpos[ZoneNum] = 0;
// 		}
// 	}
// 	return 0;
// }
 
// uint8_t PrintXYZCoords(XYZ_ZoneCoordinates_t *XYZ_ZoneCoordinates)
// {
// 	uint8_t i, j;
// 	printf("XYZ Coordinates for the target in each zone\n");
// 	for (i = 0; i < 8; i++) \
// 	{
// 		for (j = 0; j < 8; j++)
// 		{
// 			idx = (i * 8 + j);
// 			printf("%5.0f, %5.0f, %5.0f |",XYZ_ZoneCoordinates->Xpos[idx],XYZ_ZoneCoordinates->Ypos[idx],XYZ_ZoneCoordinates->Zpos[idx]);
// 		}
// 		printf("\n");
// 	}
// 	printf("\n");
 
// 	return 0;
// }