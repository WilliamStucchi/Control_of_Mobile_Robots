/*  Author: William Stucchi
    Version: 0.0.0
    Date: 21/10/2023
*/

#include "ros/ros.h"
#include "std_msgs/Int32.h"

class SystemClock {
  public:

    // constructor
    SystemClock(void);

    // this function executes the core code of this class, pubilshing the clock for timing other nodes
    void PeriodicTask(void);

    // function that keeps this node active while it is executing its core functionalities
    void run(void);

  private:
    ros::NodeHandle nh;
    ros::Publisher sc_publisher;

    // problem parameters
    float period;
    int increment_value = 0;
    int clock_value;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "system_clock");

  // start operations
  SystemClock system_clock;
  system_clock.run();

  return 0;
}

SystemClock::SystemClock(void) {
  // setup publisher
  this->sc_publisher = this->nh.advertise<std_msgs::Int32>("clock", 1);

  // get param from parameter server
  std::string paramName = "/increment_value";
  if(this->nh.getParam(paramName, this->increment_value) != true) {
      ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), paramName.c_str());
  }

  this->period = 5.0;
  this->clock_value = 0;
}

void SystemClock::PeriodicTask(void) {
  // publish clock in topic /clock
  std_msgs::Int32 message_clock;
  message_clock.data = this->clock_value;

  this->sc_publisher.publish(message_clock);

  // increment clock value
  this->clock_value += this->increment_value;
}

void SystemClock::run(void) {
  ros::Rate LoopRate(1.0/this->period);

  while(ros::ok()) {
      this->PeriodicTask();

      ros::spinOnce();

      LoopRate.sleep();
  }
}