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
#include <geometry_msgs/PoseStamped.h>   //xyz, xyzw, header
#include "Wire.h"
#include <VL53L0X.h>
#include <string>
#include <main.h>

#define FOOD_PIN D5         // QRE1113
#define LED1_PIN D0         // D0 and D4 are the built in LEDs
#define LED2_PIN D4
// #define ENC_PIN_R D11
// #define ENC_PIN_L D12

// Network info
const char* ssid     = WIFI_SSID;
const char* password = WIFI_PASS;
// Set the rosserial socket server IP address
IPAddress server(10,5,6,3);
// Set the rosserial socket server port
const uint16_t serverPort = 11411;

ros::NodeHandle nh;
// Make a chatter publisher
// std_msgs::String str_msg;
geometry_msgs::PoseStamped pose_msg;        //TODO custom message
ros::Publisher chatter("Robot", &pose_msg);

// Be polite and say hello
// char hello[13] = "hello world!";

// Robot class
ant Robot;

// Lidar sensor
VL53L0X sensor;             // VL53L0X

// Initialize Variables
float smell, viewDist, rightVel, leftVel, Hz;
bool lightTog = false;
byte address = 41; //address 0x29
String event, name = BUILD_ENV_NAME;
char cmd[10];

void adminCommands(char* cmd);

void setup()
{    
    // Use ESP8266 serial to monitor the process
    Serial.begin(74880);
    Serial.println("");
    Serial.print("Hello World! I am ");
    Serial.println(name);
    Serial.println("");

    // Wait until user is ready
    Serial.print("Press any key to begin");
    while(Serial.available() != 1);
    Serial.read();

    Serial.print("\r");
    Serial.print("Connecting to ");
    Serial.println(ssid);

    // join 12C bus and set up Lidar
    Wire.begin(4, 5); //SDA, SCL
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
    int timeout = 0;
    while (WiFi.status() != WL_CONNECTED || timeout == 500) {
        timeout++;
        delay(500);
        Serial.print(".");
        lightTog = !lightTog;
        digitalWrite(LED1_PIN, lightTog);
        digitalWrite(LED2_PIN, false);
    }

    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());

    // Set the connection to rosserial socket server
    nh.getHardware()->setConnection(server, serverPort);
    nh.initNode();

    // Another way to get IP
    // Serial.print("IP = ");
    // Serial.println(nh.getHardware()->getLocalIP());

    // Start to be polite
    nh.advertise(chatter);


}

void loop()
{
//---------------------------------------------SENSOR INPUTS--------------------------------------------------------------------

    if (sensor.timeoutOccurred()) {  //if Lidar sensor times out
        try{throw "LIDAR TIMEOUT";} catch(int E){Serial.print("AN EXCEPTION WAS THROWN: "); Serial.print(E);}     // throw exception
    }
    
    viewDist = sensor.readRangeContinuousMillimeters();     // gives Lidar distance in mm
    smell = Robot.ReadIR(FOOD_PIN);                         // returns true if food found, returns false if not

    // rightVel = analogRead(ENC_PIN_R);
    // leftVel = analogRead(ENC_PIN_L);



//----------------------------------------------CALCULATIONS--------------------------------------------------------------------

    Robot.getOdom(leftVel, rightVel);



//---------------------------------------------ROS CONNECTION--------------------------------------------------------------------

    if(WiFi.status() != WL_CONNECTED){      //connected to network
        Serial.print("DISCONNECTED FROM ");
        Serial.println(ssid);
    }


    // if (nh.connected()) {    //connected to roscore
    // Serial.println("Connected");
    // //Say hello
    // str_msg.data = hello; 
    // } 
    // else {
    //     Serial.println("Not Connected");
    // }


// TODO make custom message type with Pose and bool to indicate phereomone type
    // pose_msg.pose.position.x = x;
    // pose_msg.pose.position.y = y;
    pose_msg.pose.position.y = 0;
    pose_msg.pose.orientation.x = Robot.currentLoc.qx;
    pose_msg.pose.orientation.y = Robot.currentLoc.qy;
    pose_msg.pose.orientation.z = Robot.currentLoc.qz;
    pose_msg.pose.orientation.w = Robot.currentLoc.qw;
    pose_msg.header.frame_id = name.c_str();    //convert to const char* bc c++ doesn't get strings

    // Robot.activate   = whether the pheromones are active or not
    chatter.publish( &pose_msg );



//------------------------------------------------BEHAVIOR--------------------------------------------------------------------

    Robot.decision(event);  // decides the overall state of the robot (wandering, tracking, etc.)






//-------------------------------------------------EXTRAS--------------------------------------------------------------------

    lightTog = !lightTog;       // Toggle both LEDs to indicate activity
    digitalWrite(LED1_PIN, lightTog);
    digitalWrite(LED2_PIN, !lightTog);

    // Serial.print(rightVel); Serial.print("\t");
    // Serial.print(leftVel); Serial.print("\t");
    // Serial.print(smell); Serial.print("us\t");
    // Serial.print(viewDist); Serial.println("mm\t");

    // nh.spinOnce();
    // Loop exproximativly at 10Hz


    if(Serial.available()){         // read input werial to see if command is typed
        int read = Serial.read();
        for(int i = 0; i < sizeof(cmd); i++){
            // char letter = static_cast<char>(read);
            cmd[i] = cmd[i+1];
        }
        cmd[9] = static_cast<char>(read);
        if(cmd[9] == '/'){
            adminCommands(cmd);
        }
    }

    delay(100);
}


void adminCommands(char* cmd){       // command is typed, something is returned
    if(cmd[7] == 'i' && cmd[8] == 'p'){             // "ip/"
        Serial.println(WiFi.localIP());}
    else if(cmd[5] == 'n' && cmd[6] == 'a' && cmd[7] == 'm' && cmd[8] == 'e'){  // "name/"
        Serial.println(name);}
    else if(cmd[5] == 'r' && cmd[6] == 's' && cmd[7] == 's' && cmd[8] == 'i'){  // "rssi/"
        Serial.println(WiFi.RSSI());}
    else if(cmd[4] == 's' && cmd[5] == 't' && cmd[6] == 'a' && cmd[7] == 't' && cmd[8] == 'e'){ // "state/"
        Serial.println(event);}
    else{
        Serial.print("Command Not Found:"); 
        Serial.print(cmd[9]); Serial.print(cmd[8]); Serial.print(cmd[7]); Serial.print(cmd[6]); Serial.print(cmd[5]); Serial.print(cmd[4]); Serial.print(cmd[3]); Serial.print(cmd[2]); Serial.print(cmd[1]); Serial.print(cmd[0]); 
        Serial.println("'");
        // for(int i = sizeof(cmd)-1; i >= 0; i--){
        //     // Serial.print(i); Serial.print(cmd[i]); Serial.print(" - "); 
        //     if(cmd[i] == NULL){break;}
        //     else{Serial.print(cmd[i]);}
        // }
    }
    Serial.read();
    
}