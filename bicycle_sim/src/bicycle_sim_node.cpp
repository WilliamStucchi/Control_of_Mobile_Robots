#include "bicycle_sim/bicycle_sim.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, NAME_OF_THIS_NODE);
  
  bicycle_sim bicycle_sim_node;
   
  bicycle_sim_node.Prepare();
  
  bicycle_sim_node.RunPeriodically();
  
  bicycle_sim_node.Shutdown();
  
  return (0);
}

