#include <cstdio>
#include "VL53L0X.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"



class TofPublisher:public rclcpp ::Node{
  public:
  TofPublisher() : Node("tof") {
   tof_0_publisher_ = this->create_publisher<sensor_msgs::msg::Range>("/tof_raw", 50);
   initSensor();
   readAndPublish();
  }

  void initSensor() {
    tof_0_ = new VL53L0X();

    if (!tof_0_->openVL53L0X()) {
        // Trouble
        RCLCPP_INFO(this->get_logger(), "Unable to open tof 0");
        exit(-1) ;
    }
    tof_0_->init();
    tof_0_->setTimeout(500);
#if defined LONG_RANGE
    // lower the return signal rate limit (default is 0.25 MCPS)
    tof_0_->setSignalRateLimit(0.1);
    // increase laser pulse periods (defaults are 14 and 10 PCLKs)
    tof_0_->setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
    tof_0_->setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
#endif

    //tof_0_->setMeasurementTimingBudget(33000);
#if defined HIGH_SPEED
    // reduce timing budget to 20 ms (default is about 33 ms)
    tof_0_->setMeasurementTimingBudget(20000);
#elif defined HIGH_ACCURACY
    // increase timing budget to 200 ms
    tof_0_->setMeasurementTimingBudget(200000);
#endif

  }
  void readAndPublish() {
        // Main loop to capture and publish images
        while (rclcpp::ok()) {

            rclcpp::Time curren_time = rclcpp::Clock().now();

            int distance = tof_0_->readRangeSingleMillimeters();
            if (tof_0_->timeoutOccurred()) {
                printf("Sensor timeout!\n");
            } else {
                if(distance < 0.5) {
                    distance = 2000;
                } else if(distance > 2000) {
                    distance=2000;
                }
                sensor_msgs::msg::Range range;

                range.field_of_view = 0.017*25;
                range.min_range = 1.0;
                range.max_range = 2000.0;
                range.range = distance * 1.0;
                range.header.stamp = curren_time;
                range.header.frame_id = "tof";
                range.radiation_type = range.INFRARED;
                tof_0_publisher_->publish(range);

                // If distance > 2000, no return received; Don't print it
                if (distance < 2500 ) {
                   // RCLCPP_INFO(this->get_logger(), "\nDistance: %5d mm ", distance);
                } else {
                   // RCLCPP_INFO(this->get_logger(), ".");
                }
            }

            

            // Sleep for a while to control the publishing rate
            rclcpp::sleep_for(std::chrono::milliseconds(33));
        }
    }

  private:

    VL53L0X * tof_0_;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr tof_0_publisher_;

    
};
int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<TofPublisher>());
	rclcpp::shutdown();
    return 0;
}
