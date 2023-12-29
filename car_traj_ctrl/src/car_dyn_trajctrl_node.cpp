#include "car_traj_ctrl/car_dyn_trajctrl.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, NAME_OF_THIS_NODE);
  
  car_dyn_trajctrl car_dyn_trajctrl_node;
   
  car_dyn_trajctrl_node.Prepare();
  
  car_dyn_trajctrl_node.RunPeriodically(car_dyn_trajctrl_node.RunPeriod);
  
  car_dyn_trajctrl_node.Shutdown();
  
  return (0);
}

