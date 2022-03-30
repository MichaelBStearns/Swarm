/*
 * rosserial Publisher Example
 * Prints "hello world!"
 * This intend to connect to a Wifi Access Point 
 * and a rosserial socket server.
 * You can launch the rosserial socket server with 
 * roslaunch rosserial_server socket.launch
 * The default port is 11411
 *
 */

/*
Links:
    NodeMCU pins:
        https://randomnerdtutorials.com/esp8266-pinout-reference-gpios/
    Sensors:
        https://www.sparkfun.com/products/9454 - QRE1113 IR sensor
        https://www.pololu.com/product/2490/specs - VL53L0X Lidar sensor
*/


#include <Arduino.h>
#include <ESP8266WiFi.h>
#define ROSSERIAL_ARDUINO_TCP
#include <ros.h>
// #include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>   //x, y, theta
#include "Wire.h"
#include <VL53L0X.h>
#include "main.h"

#define FOOD_PIN D5         // QRE1113
#define LED1_PIN D0         // D0 and D4 are the built in LEDs
#define LED2_PIN D4
// #define ENC_PIN_R D11
// #define ENC_PIN_L D12

const char* ssid     = "SwarmHub";
const char* password = "The@ntH1ll";
// Set the rosserial socket server IP address
IPAddress server(10,5,6,3);
// Set the rosserial socket server port
const uint16_t serverPort = 11411;

ros::NodeHandle nh;
// Make a chatter publisher
// std_msgs::String str_msg;
geometry_msgs::PoseStamped pose_msg;
ros::Publisher chatter("Robot", &pose_msg);

// Be polite and say hello
// char hello[13] = "hello world!";

// Robot class
ant Robot;

// IR sensor
VL53L0X sensor;             // VL53L0X

int rightVel, leftVel, threshold, *xyzw;
float smell, viewDist;
bool food, found = false, lightTog = false;
byte error, address=41; //address 0x29

void setup()
{    
    // Use ESP8266 serial to monitor the process
    Serial.begin(74880);
    // Wait until user is ready
    while(Serial.available() != 1);
    Serial.read();
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);

    // join 12C bus and set time
    Wire.begin(4, 5); //SDA, SCL
    // Wire.setClock(400000); // use 400 kHz I2C
    // Wire.beginTransmission(address);
    // error = Wire.endTransmission();
    sensor.init();
    sensor.setTimeout(500);
 
    sensor.startContinuous();

    // Establish pins
    pinMode(LED1_PIN, OUTPUT);
    pinMode(LED2_PIN, OUTPUT);
    // pinMode(ENC_PIN_R, INPUT);
    // pinMode(ENC_PIN_L, INPUT);

    // Connect the ESP8266 the the wifi AP
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());

    // Set the connection to rosserial socket server
    nh.getHardware()->setConnection(server, serverPort);
    nh.initNode();

    // Another way to get IP
    Serial.print("IP = ");
    Serial.println(nh.getHardware()->getLocalIP());

    // Start to be polite
    nh.advertise(chatter);
}

void loop()
{
    if (sensor.timeoutOccurred()) {  //if Lidar sensors time out
        Serial.print(" TIMEOUT"); 
    }
    // if (nh.connected()) {
    // Serial.println("Connected");
    // Say hello
    // str_msg.data = hello; 

    // TODO make custom message type with Pose and bool to indicate phereomone type
    
    pose_msg.pose.position.x = viewDist; //just for testing
    pose_msg.pose.position.y = smell; //just for testing
    pose_msg.pose.position.y = 0;
    pose_msg.pose.orientation.x = Robot.loc.qx;   // is this how pointers work?
    pose_msg.pose.orientation.y = Robot.loc.qy;
    pose_msg.pose.orientation.z = Robot.loc.qz;
    pose_msg.pose.orientation.w = Robot.loc.qw;
    pose_msg.header.frame_id = getenv ("PATH");  //get name of environment (Robot name)

    chatter.publish( &pose_msg );

    // Serial.println(pose_msg);
    
    // } 
    // else {
        // Serial.println("Not Connected");
    // }

    // rightVel = analogRead(ENC_PIN_R);
    // leftVel = analogRead(ENC_PIN_L);

    viewDist = sensor.readRangeContinuousMillimeters();
    smell = Robot.ReadIR(FOOD_PIN);

    // Serial.print(rightVel); Serial.print("\t");
    // Serial.print(leftVel); Serial.print("\t");
    // Serial.print(smell); Serial.print("us\t");
    // Serial.print(viewDist); Serial.println("mm\t");

    // set threshold between reading when theres food and reading where there isn't
    threshold = 20;
    if(smell >= threshold){
        food = true;
        found = true;
    }
    else if(smell < threshold){
        food = false;
    }

    if(found == false){     //before finding food
        Robot.randomSearch();
    }
    else{       //after finding food
        Robot.findHelp();
    }
    Robot.getOdom(leftVel, rightVel);

    lightTog = !lightTog;
    digitalWrite(LED1_PIN, lightTog);
    digitalWrite(LED2_PIN, !lightTog);

    // nh.spinOnce();
    // Loop exproximativly at 10Hz
    delay(100);
}