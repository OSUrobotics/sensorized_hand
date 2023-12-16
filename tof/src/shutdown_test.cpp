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

#include "functions/PointCalcs.h"
#include "functions/sensor_bringup.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "hand_msgs/msg/tofzone.hpp"
#include "hand_msgs/msg/tof64.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
// #include "sensor_msgs/PointField.hpp"
// #include "sensor_msgs/point_cloud2_iterator.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node 
{
  public:
    MinimalPublisher()//VL53L7CX_Configuration Dev_in)
    : Node("minimal_publisher"), count_(0)//, Dev{Dev_in}
    {
		rclcpp::on_shutdown(std::bind( &MinimalPublisher::tof_shutdown, this));
		publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
		tof_publisher_ = this->create_publisher<hand_msgs::msg::Tof64>("tof_msg", 10);
		points_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("points", 10);
		timer_ = this->create_wall_timer(
		100ms, std::bind(&MinimalPublisher::timer_callback, this));
		
		// Setup and start the sensor
		tof_setup();
		// tof_shutdown();

		
		

		

	}
	

  private:
  	VL53L7CX_Configuration 	Dev;
	uint16_t left_sensor = 0x52;
	uint16_t right_sensor = 0x50;
	int status;
	// Create an instance of the PointCalcs class
	PointCalcs point_calc;
	sensor_msgs::msg::PointCloud2 pcl_msg_back;

	void tof_setup() 
	{
		/** 
		 * Start the TOF sensor at the provided I2C address. 
		 */
		

		// Start the left sensor
		status = sensor_bringup(Dev, left_sensor);
		if (status) {
			// Something went wrong, throw an error and shutdown node
			RCLCPP_ERROR(this->get_logger(), "Sensor %d bringup failed.", left_sensor);
			rclcpp::shutdown();
		} 
	}

	void tof_shutdown() {
		/** 
		 * Stop ranging and shutdown the sensor.
		 */
		RCLCPP_INFO(this->get_logger(), "Shutting down sensor...");
		// Shutdown the sensor and stop ranging
		vl53l7cx_stop_ranging(&Dev);
		vl53l7cx_comms_close(&Dev.platform);
		RCLCPP_INFO(this->get_logger(), "Sensor shutdown.");
	}

	void publish_tof()
	{
		int status;
		uint8_t isReady, i, j, loop;
		VL53L7CX_ResultsData 	Results;
		auto tof_all_zones = hand_msgs::msg::Tof64();
		auto tof_points = sensor_msgs::msg::PointCloud2();

		loop = 0;
		while(loop < 10)
		{
			status = vl53l7cx_check_data_ready(&Dev, &isReady);

			if(isReady)
			{
				// This gets and returns the actual distance data
				vl53l7cx_get_ranging_data(&Dev, &Results);

				/* As the sensor is set in 4x4 mode by default, we have a total 
				* of 16 zones to print. For this example, only the data of first zone are 
				* print */
				// printf("Print data no : %3u\n", &Dev.streamcount);
				for(i = 0; i < 64; i++)
				{
					printf("Zone: %3d, Status: %3u, Distance: %4d mm, Ambient per: %4d , NBtargetdetect: %4d, Signal: %8d\n",
						i,
						Results.target_status[VL53L7CX_NB_TARGET_PER_ZONE*i],
						Results.distance_mm[VL53L7CX_NB_TARGET_PER_ZONE*i],
						Results.ambient_per_spad[VL53L7CX_NB_TARGET_PER_ZONE*i],
						Results.nb_target_detected[VL53L7CX_NB_TARGET_PER_ZONE*i],
						Results.signal_per_spad[VL53L7CX_NB_TARGET_PER_ZONE*i]);
					// printf("%d", VL53L7CX_NB_TARGET_PER_ZONE);
					auto tof_zone = hand_msgs::msg::Tofzone();
					tof_zone.zone_num = i;
					tof_zone.ambient = Results.ambient_per_spad[i];
					
					
					tof_zone.num_target = Results.nb_target_detected[i];
					// printf(" %d", tof_zone.num_target);
					tof_zone.num_spad = Results.nb_spads_enabled[i];

					for (j = 0; j < VL53L7CX_NB_TARGET_PER_ZONE; j++) {
						uint16_t idx = VL53L7CX_NB_TARGET_PER_ZONE * i + j;
						tof_zone.distance[j] = Results.distance_mm[idx];
						tof_zone.status[j] = Results.target_status[idx];
						tof_zone.sigma[j] = Results.range_sigma_mm[idx];
						tof_zone.signal[j] = Results.signal_per_spad[idx];
						tof_zone.reflectance[j] = Results.reflectance[idx];
					}
					tof_all_zones.tof_array[i] = tof_zone;
				}
				printf("\n");
				break;
			}
			// printf("\n");
			loop++;
			WaitMs(&Dev.platform, 10);
		}
			tof_publisher_->publish(tof_all_zones);
			printf("\npublish!!!!\n");
			printf("Point class %i", point_calc.what);
			// pcl_msg_back = point_calc.test();
			printf("WHATTTT");
			// points_publisher_->publish(pcl_msg_back);
			pcl_msg_back = point_calc.test_process(tof_all_zones);
			points_publisher_->publish(pcl_msg_back);
	}
	
    void timer_callback()
    {	
		publish_tof();
    //   auto message = std_msgs::msg::String();
    //   message.data = "Hello, world! " + std::to_string(count_++);
    //   RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    //   publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
	rclcpp::Publisher<hand_msgs::msg::Tof64>::SharedPtr tof_publisher_;
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_publisher_;
    size_t count_;
};

// VL53L7CX_Configuration setup_sensor(){

// }

int main(int argc, char * argv[])
{

//   PointCalcs pointssss;
//   printf("\n Hi %i \n", pointssss.what);	
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}