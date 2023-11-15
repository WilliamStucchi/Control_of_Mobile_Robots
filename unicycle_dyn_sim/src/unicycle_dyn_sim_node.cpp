#include "unicycle_dyn_sim/unicycle_dyn_sim.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, NAME_OF_THIS_NODE);
  
  unicycle_dyn_sim unicycle_dyn_sim_node;
   
  unicycle_dyn_sim_node.Prepare();
  
  unicycle_dyn_sim_node.RunPeriodically();
  
  unicycle_dyn_sim_node.Shutdown();
  
  return (0);
}

