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
#include <geometry_msgs/PoseStamped.h> //xyz, xyzw, header
#include <swarm_msgs/Grid.h>
#include "Wire.h"
#include <VL53L0X.h>
#include <string>
#include <main.h>

#define FOOD_PIN D5 // QRE1113
#define LED1_PIN D0 // D0 and D4 are the built in LEDs
#define LED2_PIN D4
// #define ENC_PIN_R D11
// #define ENC_PIN_L D12

// Network info
const char *ssid = WIFI_SSID;
const char *password = WIFI_PASS;
// Set the rosserial socket server IP address
IPAddress server(10, 5, 6, 3);
// Set the rosserial socket server port

const uint16_t serverPort = BUILD_ENV_PORT;

// Robot class
ant Robot;

// Lidar sensor
VL53L0X sensor; // VL53L0X


ros::NodeHandle nh;
// Make a chatter publisher
// std_msgs::String str_msg;
geometry_msgs::PoseStamped pose_msg; // TODO custom message?

void grid_msg(const swarm_msgs::Grid& msg){
    Robot.currentPher[0] = msg.column[Robot.currentLoc.x].row[Robot.currentLoc.y].pheromones[0];
    Robot.currentPher[1] = msg.column[Robot.currentLoc.x].row[Robot.currentLoc.y].pheromones[1];
    Robot.currentPher[2] = msg.column[Robot.currentLoc.x].row[Robot.currentLoc.y].pheromones[2];
    Robot.currentPher[3] = msg.column[Robot.currentLoc.x].row[Robot.currentLoc.y].pheromones[3];
    Robot.currentPher[4] = msg.column[Robot.currentLoc.x].row[Robot.currentLoc.y].pheromones[4];
    // Serial.println(msg.column);
    // Serial.println("Pheromone!");
}
ros::Publisher chatter("/Pheromones_Write", &pose_msg);
ros::Subscriber<swarm_msgs::Grid> sub("/Pheromones_Read", &grid_msg);

// Initialize Variables
float smell, viewDist, rightVel, leftVel, Hz;
bool lightTog = false;
int timeout = 0;
byte address = 41; // address 0x29
String event, name = BUILD_ENV_NAME;
char cmd[10];

void adminCommands(char *cmd);

void setup()
{
    // Use ESP8266 serial to monitor the process
    Serial.begin(74880);
    Serial.println("");
    Serial.print("Hello World! I am ");
    Serial.println(name);
    Serial.println("");

    // Wait until user is ready
    // Serial.print("Press any key to begin");
    // while (Serial.available() != 1);
    // Serial.read();

    Serial.print("\r");
    Serial.print("Connecting to ");
    Serial.println(ssid);

    // join 12C bus and set up Lidar
    Wire.begin(4, 5); // SDA, SCL
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
    while (WiFi.status() != WL_CONNECTED && timeout <= 60)
    {
        timeout++;
        delay(500);
        Serial.print(".");

        lightTog = !lightTog;
        digitalWrite(LED1_PIN, lightTog);
        digitalWrite(LED2_PIN, true);
    }

    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());

    // Set the connection to rosserial socket server
    nh.getHardware()->setConnection(server, serverPort);
    nh.initNode();
    // Start to be polite
    nh.advertise(chatter);
    // Start asking questions
    nh.subscribe(sub);
    delay(10000);
}

void loop()
{
    /* #region ---------------------------------------------SENSOR--INPUTS-------------------------------------------------------------------*/

    if (sensor.timeoutOccurred())
    { // if Lidar sensor times out
        try
        {
            throw "LIDAR TIMEOUT";
        }
        catch (int E)
        {
            Serial.print("AN EXCEPTION WAS THROWN: ");
            Serial.print(E);
        } // throw exception
    }

    viewDist = sensor.readRangeContinuousMillimeters(); // gives Lidar distance in mm
    smell = Robot.ReadIR(FOOD_PIN);                     // returns true if food found, returns false if not

    // rightVel = analogRead(ENC_PIN_R);
    // leftVel = analogRead(ENC_PIN_L);

    /* #endregion */
    /* #region ----------------------------------------------CALCULATIONS--------------------------------------------------------------------*/

    Robot.getOdom(leftVel, rightVel);

    /* #endregion */
    /* #region ---------------------------------------------ROS-CONNECTION-------------------------------------------------------------------*/

    if(WiFi.status() == WL_CONNECTED && nh.connected()){    // connected to Wifi and roscore
        chatter.publish(&pose_msg);
        digitalWrite(LED1_PIN, false);
        digitalWrite(LED2_PIN, true);

    }
    else if (WiFi.status() != WL_CONNECTED){                // not connected to network
        Serial.print("DISCONNECTED FROM ");  Serial.println(ssid);
        WiFi.reconnect();
        timeout = 0;
        while (WiFi.status() != WL_CONNECTED && timeout <= 20)
        {
            timeout++;
            delay(500);
            Serial.print(".");

            lightTog = !lightTog;
            digitalWrite(LED1_PIN, lightTog);               // slow blink
            digitalWrite(LED2_PIN, true);
        }
        Serial.println("");
    }
    else if (!nh.connected()){                              // only not connected to roscore
        Serial.println("DISCONNECTED FROM ROSCORE");
        nh.advertise(chatter);
        timeout = 0;
        while (!nh.connected() && timeout <= 10){
            timeout++;
            delay(250);
            Serial.print(".");

            lightTog = !lightTog;
            digitalWrite(LED1_PIN, lightTog);               // faster blink
            digitalWrite(LED2_PIN, true);
        }
        Serial.println("");
    }

    Robot.active = 1;
    // TODO make custom message type with Pose and bool to indicate phereomone type
    pose_msg.pose.position.x = Robot.currentLoc.x;
    pose_msg.pose.position.y = Robot.currentLoc.y;
    pose_msg.pose.position.y = Robot.currentLoc.z;
    pose_msg.pose.orientation.x = Robot.currentLoc.qx;
    pose_msg.pose.orientation.y = Robot.currentLoc.qy;
    pose_msg.pose.orientation.z = Robot.currentLoc.qz;
    pose_msg.pose.orientation.w = Robot.currentLoc.qw;
    pose_msg.header.frame_id = name.c_str();                // convert to const char* bc c++ doesn't do strings
    pose_msg.header.stamp.sec = Robot.active;               // hijack time and replace with pheromone activation


    // Light front LED solid to indicate connected

    /* #endregion */
    /* #region ------------------------------------------------BEHAVIOR----------------------------------------------------------------------*/

    Robot.decision(event); // decides the overall state of the robot (wandering, tracking, etc.)


    /* #endregion */
    /* #region -------------------------------------------------EXTRAS-----------------------------------------------------------------------*/

    // Serial.print(rightVel); Serial.print("\t");
    // Serial.print(leftVel); Serial.print("\t");
    // Serial.print(smell); Serial.print("us\t");
    // Serial.print(viewDist); Serial.println("mm\t");

    // nh.spinOnce();
    // Loop exproximativly at 10Hz

    if (Serial.available())
    { // read input serial to see if command is typed
        int read = Serial.read();
        for (int i = 0; abs(i) < sizeof(cmd) - 1; i++)
        {
            // char letter = static_cast<char>(read);
            cmd[i] = cmd[i + 1];
        }
        cmd[9] = static_cast<char>(read);
        if (cmd[9] == '/')
        {
            adminCommands(cmd);
        }
    }
    /* #endregion */
    /* #region ---------------------------------------------------END------------------------------------------------------------------------*/
    delay(100);
    nh.spinOnce();
}
    /* #endregion */


void adminCommands(char *cmd)
{ // command is typed, something is returned
    if (cmd[7] == 'i' && cmd[8] == 'p')
    { // "ip/"
        Serial.println(WiFi.localIP());
    }
    else if (cmd[5] == 'n' && cmd[6] == 'a' && cmd[7] == 'm' && cmd[8] == 'e')
    { // "name/"
        Serial.println(name);
    }
    else if (cmd[5] == 'r' && cmd[6] == 's' && cmd[7] == 's' && cmd[8] == 'i')
    { // "rssi/"
        Serial.println(WiFi.RSSI());
    }
    else if (cmd[4] == 's' && cmd[5] == 't' && cmd[6] == 'a' && cmd[7] == 't' && cmd[8] == 'e')
    { // "state/"
        Serial.println(event);
    }
    else if (cmd[5] == 'p' && cmd[6] == 'o' && cmd[7] == 'r' && cmd[8] == 't')
    { // "state/"
        Serial.println(serverPort);
    }
    else
    {
        Serial.print("Command Not Found:");
        Serial.print(cmd[9]);
        Serial.print(cmd[8]);
        Serial.print(cmd[7]);
        Serial.print(cmd[6]);
        Serial.print(cmd[5]);
        Serial.print(cmd[4]);
        Serial.print(cmd[3]);
        Serial.print(cmd[2]);
        Serial.print(cmd[1]);
        Serial.print(cmd[0]);
        Serial.println("'");
        // for(int i = sizeof(cmd)-1; i >= 0; i--){
        //     // Serial.print(i); Serial.print(cmd[i]); Serial.print(" - ");
        //     if(cmd[i] == NULL){break;}
        //     else{Serial.print(cmd[i]);}
        // }
    }
    Serial.read();
}