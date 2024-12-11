#include"ros_node.h"
#include <memory>
#include <string>
#include <utility>
#include "common/public.h"

int main(int argc, char **argv)
{
   ros::init(argc, argv, "olelidar");

   ros::NodeHandle pnh("~");
   TNodeCheck* nodecheck=new TNodeCheck(&pnh, "node_rate");
	nodecheck->Find("node_rate")->SetLimit(10);

   olelidar::Ros_node node(pnh);
   ros::spin();
}