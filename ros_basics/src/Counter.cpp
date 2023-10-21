/*  Author: William Stucchi
    Version: 0.0.0
    Date: 21/10/2023
*/

#include "ros/ros.h"
#include "std_msgs/Int32.h"

class Counter {
  public:

    // constructor
    Counter(void);

    // this function is called when a message is published on topic /clock
    void execute_Clock_Callback(const std_msgs::Int32::ConstPtr &msg);

    // this function must be called every time an object of this class is created, after its creation
    void run(void);

  private:
    ros::NodeHandle nh;
    ros::Subscriber ctr_subscriber;
    ros::Publisher ctr_publisher;

    // problem parameters
    int C_i;
    int C_f;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "counte");

  // start operations
  Counter counter_node;
  counter_node.run();

  return 0;
}

Counter::Counter() {
  // setup publisher and subscriber 
  this->ctr_publisher = this->nh.advertise<std_msgs::Int32>("value", 1);
  this->ctr_subscriber = this->nh.subscribe("clock", 1, &Counter::execute_Clock_Callback, this);

  this->C_i = 0;
  this->C_f = 10;
}

void Counter::execute_Clock_Callback(const std_msgs::Int32::ConstPtr &msg) {
  // increment counter
  this->C_i = this->C_i + 1;

  // when condition satisfied, we want ot publish our counter status
  if(this->C_i == this->C_f) {
    int temp = this->C_i;

    // include this next line if you want to repeat the counting after C_i reaches C_f
    // this->C_i = 0.0;

    // publish message on topic /value
    std_msgs::Int32 message_value;
    message_value.data = temp;

    this->ctr_publisher.publish(message_value);
  }
}

void Counter::run() {
  ros::spin();
}