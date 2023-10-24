#include <chrono>
#include <functional>
#include <memory>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <signal.h>
#include <dlfcn.h>
extern "C"
{
  #include "uld-driver/inc/vl53l7cx_api.h"
  // #include "uld-driver/platform/platform.h"
  

}

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "hand_msgs/msg/tofzone.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node 
{
  public:
  
    MinimalPublisher(VL53L7CX_Configuration Dev_in)
    : Node("minimal_publisher"), count_(0), Dev{Dev_in}
    {
	
	static uint8_t 				loop, isReady, i;
	static VL53L7CX_ResultsData 	Results;		/* Results data from VL53L7CX */
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
	  tof_publisher_ = this->create_publisher<hand_msgs::msg::Tofzone>("tof_msg", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
    

	
	
	printf("wtf\n");
	tof_setup();
	}
  private:
  	VL53L7CX_Configuration 	Dev;
	void tof_setup() 
	{
		// TODO: Update this method to be bool and return false if setup failure
		int status;
		uint8_t isAlive;
		// /* Initialize channel com */
		status = vl53l7cx_comms_init(&Dev.platform);
		if(status) {printf("VL53L7CX comms init failed\n");} 
		else {printf("VL53L7CX comms success\n");}
		// Check if sensor connected
		status = vl53l7cx_is_alive(&Dev, &isAlive);
		if(!isAlive || status)
		{printf("VL53L7CX not detected at requested address\n");}
		// Wakeup sensor
		status = vl53l7cx_set_power_mode(&Dev, VL53L7CX_POWER_MODE_WAKEUP);
		if(status)
		{printf("VL53L7CX wakeup failed\n");}
		// Initialize sensor
		status = vl53l7cx_init(&Dev);
		if(status)
		{printf("VL53L7CX ULD Loading failed\n");}
		printf("VL53L7CX ULD ready ! (Version : %s)\n",
				VL53L7CX_API_REVISION);
		// Set the ranging mode to continuous 
		status = vl53l7cx_set_ranging_mode(&Dev, VL53L7CX_RANGING_MODE_CONTINUOUS);
		// Set the ranging frequency to 15 Hz (for 8x8 zone can be 1-15, for 4x4 1-60)
		vl53l7cx_set_ranging_frequency_hz(&Dev, 15);

		status = vl53l7cx_start_ranging(&Dev);
	}

	// 	/*********************************/
	// 	/*         Ranging loop          */
	// 	/*********************************/
	// 	// Paramters are in vl53l7cx_api.h
	// 	// Set the ranging mode to continuous 
	// 	status = vl53l7cx_set_ranging_mode(&Dev, VL53L7CX_RANGING_MODE_CONTINUOUS);
	// 	// Set the ranging frequency to 15 Hz (for 8x8 zone can be 1-15, for 4x4 1-60)
	// 	vl53l7cx_set_ranging_frequency_hz(&Dev, 15);

	// 	status = vl53l7cx_start_ranging(&Dev);

	// 	// return Dev;
	// }

	
    void timer_callback()
    {
		int status;
		uint8_t isReady, i, loop;
		VL53L7CX_ResultsData 	Results;

			loop = 0;
	while(loop < 10)
	{
		status = vl53l7cx_check_data_ready(&Dev, &isReady);

		if(isReady){printf("ready!!!!");
			// This gets and returns the actual distance data
			vl53l7cx_get_ranging_data(&Dev, &Results);

			/* As the sensor is set in 4x4 mode by default, we have a total 
			 * of 16 zones to print. For this example, only the data of first zone are 
			 * print */
			printf("Print data no : %3u\n", &Dev.streamcount);
			for(i = 0; i < 16; i++)
			{
				printf("Zone: %3d, Status: %3u, Distance: %4d mm, Ambient per: %4d , NBtargetdetect: %4d, Signal: %8d\n",
					i,
					Results.target_status[VL53L7CX_NB_TARGET_PER_ZONE*i],
					Results.distance_mm[VL53L7CX_NB_TARGET_PER_ZONE*i],
					Results.ambient_per_spad[VL53L7CX_NB_TARGET_PER_ZONE*i],
					Results.nb_target_detected[VL53L7CX_NB_TARGET_PER_ZONE*i],
					Results.signal_per_spad[VL53L7CX_NB_TARGET_PER_ZONE*i]);
				//printf("%d", VL53L7CX_NB_TARGET_PER_ZONE);
				auto tof_zone = hand_msgs::msg::Tofzone();
				tof_zone.distance = Results.distance_mm[VL53L7CX_NB_TARGET_PER_ZONE*i];
				tof_zone.status = Results.target_status[VL53L7CX_NB_TARGET_PER_ZONE*i];
				tof_zone.sigma = Results.range_sigma_mm[VL53L7CX_NB_TARGET_PER_ZONE*i];
				tof_zone.ambient = Results.ambient_per_spad[VL53L7CX_NB_TARGET_PER_ZONE*i];
				tof_zone.num_target = Results.nb_target_detected[VL53L7CX_NB_TARGET_PER_ZONE*i];
				tof_zone.num_spad = Results.nb_spads_enabled[VL53L7CX_NB_TARGET_PER_ZONE*i];
				tof_zone.signal = Results.signal_per_spad[VL53L7CX_NB_TARGET_PER_ZONE*i];
				tof_zone.reflectance = Results.reflectance[VL53L7CX_NB_TARGET_PER_ZONE*i];
				tof_publisher_->publish(tof_zone);
				printf("publish!!!!");
			}

		}
			printf("\n");
			loop++;
			WaitMs(&Dev.platform, 5);
		}

		/* Wait a few ms to avoid too high polling (function in platform
		 * file, not in API) */
		// WaitMs(&Dev.platform, 5);
	

      auto message = std_msgs::msg::String();
      message.data = "Hello, world! " + std::to_string(count_++);
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
	rclcpp::Publisher<hand_msgs::msg::Tofzone>::SharedPtr tof_publisher_;
	
    size_t count_;
};

// VL53L7CX_Configuration setup_sensor(){

// }

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

	// status = vl53l7cx_stop_ranging(&Dev);
	// printf("End of ULD demo\n");

	// status = vl53l7cx_set_power_mode(&Dev, VL53L7CX_POWER_MODE_SLEEP);
	// if(status)
	// {
	// 	printf("VL53L7CX sleep failed\n");
	// 	return status;
	// }

	// printf("Starting examples with ULD version %s\n", VL53L7CX_API_REVISION);
VL53L7CX_Configuration 	Dev;
  rclcpp::spin(std::make_shared<MinimalPublisher>(Dev));
  rclcpp::shutdown();
  return 0;
}