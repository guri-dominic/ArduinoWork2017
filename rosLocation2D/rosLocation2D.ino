#include <ArduinoHardware.h>
#include <ros.h>

#define TIME_STAMPED
#define SIMULATE
#define ZERO        0

///////////////////////////////////////////////////////////////////////////
#ifdef TIME_STAMPED
    #include <geometry_msgs/PointStamped.h>
    geometry_msgs::PointStamped msg;
    //      msg.point.x
    //      msg.point.y
    //      msg.point.z
    char frame_id[] = "/quad_loc";
#else
    #include <geometry_msgs/Point.h>
    geometry_msgs::Point32 msg;
    //      msg.point.x
    //      msg.point.y
    //      msg.point.z
#endif
///////////////////////////////////////////////////////////////////////////
ros::NodeHandle nh;
ros::Publisher pubLoc("/pozyx_location2D", &msg);

double  xloc = 0;
double  yloc = 0;
double  zloc = ZERO;
// simulation
double x_inc = 2;

///////////////////////////////////////////////////////////////////////////
void setup()
{
    nh.initNode();
    nh.advertise(pubLoc);
} 
///////////////////////////////////////////////////////////////////////////
void loop()
{
    updateMessage();
    pubLoc.publish(&msg);
    nh.spinOnce();
}
///////////////////////////////////////////////////////////////////////////
void getLocation()
{
    #ifndef SIMULATE
        // Pozyx Localization
    #else
        // Simulated Localization
        delay(50);
        xloc += x_inc;
        yloc = 1000 - xloc;
        x_inc = (xloc >= 1000)||(xloc <= 0) ? (-1*x_inc) : x_inc;
    #endif
    
    msg.point.x = xloc;
    msg.point.y = yloc;
    msg.point.z = zloc;
}

///////////////////////////////////////////////////////////////////////////
#ifdef TIME_STAMPED
//geometry_msgs::TransformStamped t;
//geometry_msgs::PointStamped msg;
//      msg.point.x
//      msg.point.y
//      msg.point.z
void updateMessage()
{
    msg.header.frame_id = frame_id;
    getLocation();
    msg.header.stamp = nh.now();
}
///////////////////////////////////////////////////////////////////////////
#else
//geometry_msgs::Point32 msg;
//      msg.point.x
//      msg.point.y
//      msg.point.z
void updateMessage()
{
    getLocation();
}
#endif

///////////////////////////////////////////////////////////////////////////
