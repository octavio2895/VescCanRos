#include "../vesc_can_interface/src/vesc_can_interface.h" 
#include <cstddef>
#include <iostream>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <vesc_msgs/VescStateStamped.h>

vesc_can_driver::VescCanInterface vesc_dev;
ros::Subscriber duty_cycle_sub_;
void dutyCycleCallbackCan(const std_msgs::Float64::ConstPtr &duty_cycle) 
{
  printf("Setting DC\n");
    // vesc_can_.setDutyCycle(duty_cycle_limit_.clip(duty_cycle->data));
    // vesc_can_.setDutyCycle(duty_cycle->data);
  vesc_dev.setDutyCycle(duty_cycle->data);

}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "vesc_driver_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  duty_cycle_sub_ = nh.subscribe("commands/motor/duty_cycle", 10, &dutyCycleCallbackCan);
  std::cout << "Started demo, creating instace" << std::endl;
  vesc_dev.host_id = 0x78;
  std::cout << "Init thread" << std::endl;
  vesc_dev.start(0x02);
  printf("Host id:%d\n", vesc_dev.host_id);
  ros::spin();
  ROS_INFO_STREAM("stopping VESC driver");
  vesc_dev.stop();

  return 0;
}
