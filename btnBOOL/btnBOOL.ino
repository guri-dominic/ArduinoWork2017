
#include <ros.h>
#include <std_msgs/Bool.h>


ros::NodeHandle nh;

std_msgs::Bool pushed_msg;
ros::Publisher pub_button("disturbance", &pushed_msg);

const int button_pin = 7;
const int led_pin = 13;

bool oldBtnReading;
long last_debounce_time=0;
long debounce_delay=50;
bool published = true;

void setup()
{
  nh.initNode();
  nh.advertise(pub_button);
  
  //initialize an LED output pin 
  //and a input pin for our push button
  pinMode(led_pin, OUTPUT);
  pinMode(button_pin, INPUT);
  //pinMode(button_pin, INPUT_PULLUP);
  
  //Enable the pullup resistor on the button
  digitalWrite(button_pin, HIGH);
  
  //The button is a normally button
  oldBtnReading = ! digitalRead(button_pin);
 
}

void loop()
{
  
  bool btnReading = ! digitalRead(button_pin);

  // set Publish flag for change in btnReading signal
  if (oldBtnReading!= btnReading){
      last_debounce_time = millis();
      published = false;
  }
  
  // Debounce (With 50ms delay)
  // + Change in btnReading
  if ( !published && (millis() - last_debounce_time)  > debounce_delay) {
    digitalWrite(led_pin, btnReading);
    pushed_msg.data = btnReading;
    pub_button.publish(&pushed_msg);
    published = true;
  }

  oldBtnReading = btnReading;
  nh.spinOnce();
}
