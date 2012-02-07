#include <ros/ros.h>
#include <openraveros/env_loadscene.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "openraveclient");
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<openraveros::env_loadscene>("openrave/env_loadscene");
    openraveros::env_loadscene srv;
    srv.request.filename = "data/lab1.env.xml";
    srv.request.resetscene = 1;
    if( !client.call(srv) ) {
        ROS_ERROR("Failed to call service env_loadscene");
        return 1;
    }
    return 0;
}
