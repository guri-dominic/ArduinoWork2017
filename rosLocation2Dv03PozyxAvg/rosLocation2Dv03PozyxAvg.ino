#include <ArduinoHardware.h>
#include <ros.h>
#include <Pozyx.h>
#include <Pozyx_definitions.h>
#include <Wire.h>

//#include <MovingAverage.h>

//#include <SoftwareSerial.h>
//SoftwareSerial Serial1(4, 5);

///////////////////////////////////////////////////////////////////////////
#define TIME_STAMPED
// #define SIMULATE
///////////////////////////////////////////////////////////////////////////
#define ANCHOR01_ID        0x6045
#define ANCHOR02_ID        0x6053
#define ANCHOR03_ID        0x6055
#define ANCHOR04_ID        0x605E
#define NUM_ANCHORS 4
#define ZERO                0.0
/////////////////////////////////////////////////////////////////////////////////////
uint16_t anchors[4] = { ANCHOR01_ID, // (0,0)
                        ANCHOR02_ID, // x-axis
                        ANCHOR03_ID, // y-axis
                        ANCHOR04_ID};

int32_t anchors_x[NUM_ANCHORS] = {0,     
                                  2820, 
                                  0,     
                                  4572};    // anchor x-coorindates in mm (horizontal)
                                  
int32_t anchors_y[NUM_ANCHORS] = {0,
                                  0,     
                                  3810, 
                                  2972};    // anchor y-coordinates in mm (vertical)

int32_t start_height = 622;
int32_t heights[NUM_ANCHORS] =   {-1470, 
                                  -1400, 
                                  -2286, 
                                  -2108};    // anchor z-coordinates in mm (1.4m above vehicle's starting altitude)
/////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////

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
//////////////////////////////////////////////////////////////////////////
//class NewHardware : public ArduinoHardware
//{
//  public:
//  NewHardware():ArduinoHardware(&Serial1, 57600){};
//};
//
//ros::NodeHandle_<NewHardware>  nh;
ros::NodeHandle nh;
ros::Publisher pubLoc("/location_pozyx", &msg);

double  xloc = 0;
double  yloc = 0;
double  zloc = -1;
// simulation
double x_inc = 2;

//MovingAverage xlocAvg(0.1f);
//MovingAverage ylocAvg(0.1f);

///////////////////////////////////////////////////////////////////////////
// 
void setup()
{
    Serial1.begin(57600);
    
    if(Pozyx.begin() == POZYX_FAILURE)
    {
        Serial1.println(F("ERROR: Unable to connect to POZYX shield"));
        Serial1.println(F("Reset required"));
        delay(100);
        abort();
    }
    else
    {
        Serial1.println("Initialization Complete!");
    }
    
    // clear all previous devices in the device list
    Pozyx.clearDevices();
    PozyxClass::setPositionAlgorithm(POZYX_POS_ALG_LS, POZYX_3D);
    SetAnchorsManual();
    Serial1.println("Anchor Configuration Complete!");
    delay(1000);

//    xlocAvg.reset(0.0); ylocAvg.reset(0.0);
    
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

void localize()
{
    #ifndef SIMULATE
        // Pozyx Localization
        // pozyx_localize();
        coordinates_t position; 
        if (Pozyx.doPositioning(&position, POZYX_2_5D) == POZYX_SUCCESS)
        {
            xloc = position.x;
            yloc = position.y;
        }
        else
        {
            Serial1.println(F("Status Fail!"));
        }
    #else
        // Simulated Localization
        delay(50);
        xloc += x_inc;
        yloc = 1000 - xloc;
        x_inc = (xloc >= 1000)||(xloc <= 0) ? (-1*x_inc) : x_inc;
    #endif
    printCoordinates();

//    xloc = xlocAvg.update(xloc);
//    yloc = ylocAvg.update(yloc);
    
    msg.point.x = xloc/1000.0;
    msg.point.y = yloc/1000.0;
    msg.point.z = ZERO;
}

#ifdef TIME_STAMPED
//geometry_msgs::TransformStamped t;
//geometry_msgs::PointStamped msg;
//      msg.point.x
//      msg.point.y
//      msg.point.z
void updateMessage()
{
    msg.header.frame_id = frame_id;
    localize();
    msg.header.stamp = nh.now();
}
#else
//geometry_msgs::Point32 msg;
//      msg.point.x
//      msg.point.y
//      msg.point.z
void updateMessage()
{
    localize();
}
#endif

/////////////////////////////////////////////////////////////////////////////////////
//void pozyx_localize(coordinates_t *position)
//{
//    
//}


/////////////////////////////////////////////////////////////////////////////////////
// function to print the coordinates to the serial monitor
void printCoordinates()
{
  Serial1.print(xloc);
  Serial1.print(", ");
  Serial1.println(yloc);
}

/////////////////////////////////////////////////////////////////////////////////////
//function to manually set the anchor coordinates
void SetAnchorsManual()
{
    int i=0;
    for(i=0; i<NUM_ANCHORS; i++)
    {
        device_coordinates_t anchor;
        anchor.network_id = anchors[i];
        anchor.flag = 0x1; 
        anchor.pos.x = anchors_x[i];
        anchor.pos.y = anchors_y[i];
        anchor.pos.z = heights[i];
        Pozyx.addDevice(anchor);
    }
}


