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
/*                      NodeMCU
                    /----------------\
   USED (Enc2) - A0 | ADC0    GPIO16 | D0 - USED (LED)
                    | RES      GPIO5 | D1 - USED (SCL-Lidar)        
                    | RES      GPIO4 | D2 - USED (SDA-Lidar)    
  USED (Enc1) - SD3 | GPIO10   GPIO0 | D3 - UNUSED
                    | GPIO9    GPIO2 | D4 - USED (LED)    
                    | MOSI      3.3V | 
                    | CS         GND | 
                    | MISO    GPIO14 | D5 - USED (IR)
                    | SCLK    GPIO12 | D6 - USED (Motor Control)  
                    | GND     GPIO13 | D7 - USED (Motor Control)  
                    | 3.3V    GPIO15 | D8 - USED (Motor Control)  
                    | EN       GPIO3 | RX - USED (Motor Control) 
                    | RST      GPIO1 | TX
                    | GND        GND |    
                    | Vin       3.3V |    
                    \------||||------/
                           ||||
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
// #include <std_msgs/String.h>D4
#include <swarm_msgs/Plus.h>    //Header, Pose, bool, string, float32[5]
#include <swarm_msgs/Grid.h>    
#include "Wire.h"
#include <VL53L0X.h>
#include <string>
#include <main.h>
      
#define LED1_PIN D0             // D0 and D4 are the built in LEDs
// #define LED2_PIN D4

#define FOOD_PIN D5             // QRE1113

#define motorR_back D3          // Motor Control
#define motorR_for D4
#define motorL_back D8
#define motorL_for D6

#define ENC_PIN_R 10 //SD3      // Encoders
#define ENC_PIN_L D7

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
swarm_msgs::Plus pose_msg; // TODO custom message?
swarm_msgs::Plus pose_msg_prev;
void sub_msg(const swarm_msgs::Grid& msg){
    // read pheromones in the robot's current square
    Robot.currentPher[0] = msg.column[(int)Robot.currentLoc.Grid.x].row[(int)Robot.currentLoc.Grid.y].pheromones[0];
    Robot.currentPher[1] = msg.column[(int)Robot.currentLoc.Grid.x].row[(int)Robot.currentLoc.Grid.y].pheromones[1];
    Robot.currentPher[2] = msg.column[(int)Robot.currentLoc.Grid.x].row[(int)Robot.currentLoc.Grid.y].pheromones[2];
    Robot.currentPher[3] = msg.column[(int)Robot.currentLoc.Grid.x].row[(int)Robot.currentLoc.Grid.y].pheromones[3];
    Robot.currentPher[4] = msg.column[(int)Robot.currentLoc.Grid.x].row[(int)Robot.currentLoc.Grid.y].pheromones[4];
}

ros::Publisher chatter("/Pheromones_Write", &pose_msg);
ros::Subscriber<swarm_msgs::Grid> smell("/Pheromones_Read", &sub_msg);

// Initialize Variables
float scent, right, left, Hz;
bool lightTog = false;
int timeout = 0, *wheels, timer = 0;
byte address = 41; // address 0x29
String state = "ROAM";  // "ROAM", "FOUND_FOOD", "GET_HELP", "FOLLOW_TRAIL"
char cmd[10];
struct Coords vels;

void adminCommands(char *cmd);
void IRAM_ATTR read_encL(void);
void IRAM_ATTR read_encR(void);

void setup()
{
    // Use ESP8266 serial to monitor the process
    Serial.begin(74880);
    Serial.println("");
    Serial.print("Hello World! I am ");
    Serial.println(Robot.name);
    Serial.println("");

    // Wait until user is ready
    // Serial.print("Press any key to begin");
    // while (Serial.available() != 1);
    // Serial.read();

    Serial.print("\r");
    Serial.print("Connecting to ");
    Serial.println(ssid);
    Serial.print("on port ");
    Serial.println(serverPort);

    // join 12C bus and set up Lidar
    Wire.begin(4, 5); // SDA, SCL
    sensor.init();
    sensor.setTimeout(500);
    sensor.startContinuous();

    // Establish pins
    pinMode(LED1_PIN, OUTPUT);
    // pinMode(LED2_PIN, OUTPUT);
    pinMode(FOOD_PIN, OUTPUT);

    pinMode(motorL_for, OUTPUT);
    pinMode(motorL_back, OUTPUT);
    pinMode(motorR_for, OUTPUT);
    pinMode(motorR_back, OUTPUT);
    
    pinMode(ENC_PIN_R, INPUT);
    pinMode(ENC_PIN_L, INPUT);

    attachInterrupt(digitalPinToInterrupt(ENC_PIN_L), read_encL, FALLING);
    attachInterrupt(digitalPinToInterrupt(ENC_PIN_R), read_encR, FALLING);

    // Connect the ESP8266 the the wifi AP
    WiFi.begin(ssid, password);
     while (WiFi.status() != WL_CONNECTED && timeout <= 10)
     {
        timeout++;
        delay(500);
        Serial.print(".");
        lightTog = !lightTog;
        digitalWrite(LED1_PIN, lightTog);
        //digitalWrite(LED2_PIN, true);
     }

    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    // Serial.println(WiFi.localIP());

    // Set the connection to rosserial socket server
    nh.getHardware()->setConnection(server, serverPort);
    nh.initNode();
    // Start to be polite
    nh.advertise(chatter);
    // Start asking questions
    nh.subscribe(smell);
    // delay(20000);
    // Robot.calibrateIR(Robot.ReadIR(FOOD_PIN));
}

void loop()
{
    /* #region ---------------------------------------------SENSOR--INPUTS-------------------------------------------------------------------*/

    if (sensor.timeoutOccurred())   // if Lidar sensor times out
    {try{throw "LIDAR TIMEOUT";} catch (int E){Serial.print("AN EXCEPTION WAS THROWN: ");Serial.print(E);}} // throw exception

    Robot.viewDist = sensor.readRangeContinuousMillimeters(); // gives Lidar distance in mm
    scent = Robot.ReadIR(FOOD_PIN);                     // returns true if food found, returns false if not

    // right = digitalRead(ENC_PIN_R);
    // left = map(analogRead(ENC_PIN_L), 1, 1024, 0, 1);

    /* #endregion */
    /* #region ----------------------------------------------CALCULATIONS--------------------------------------------------------------------*/

    // convert pos in mm to grid pos
    Coords grid = Robot.pos_to_Grid(Robot.currentLoc.Pos.x, Robot.currentLoc.Pos.y);
    Robot.currentLoc.Grid.x = grid.x;
    Robot.currentLoc.Grid.y = grid.y;

    Robot.euler_to_quarternion();

    // Robot.locateObstacle(Robot.viewDist);

    // Robot.getOdom(left, right);

    /* #endregion */
    /* #region ---------------------------------------------ROS-CONNECTION-------------------------------------------------------------------*/

    pose_msg.pose.position.x = Robot.currentLoc.Pos.x;      // std_msgs/Pose (minus z)
    pose_msg.pose.position.y = Robot.currentLoc.Pos.y;
    pose_msg.pose.orientation.x = Robot.currentLoc.qx;
    pose_msg.pose.orientation.y = Robot.currentLoc.qy;
    pose_msg.pose.orientation.z = Robot.currentLoc.qz;
    pose_msg.pose.orientation.w = Robot.currentLoc.qw;
    pose_msg.header.frame_id = Robot.name.c_str();          // Robot name (convert to const char* bc c++ doesn't do strings)
    pose_msg.pheromone = Robot.active;                      // pheromone activation
    pose_msg.state = state.c_str();                         // state of robot for reference
    // pose_msg.display[0] = ;                              // if anything to display wirelessly (bugfixing)
    // pose_msg.display[1] = ;
    // pose_msg.display[2] = ;
    // pose_msg.display[3] = ;
    // pose_msg.display[4] = ;


    /* if(WiFi.status() == WL_CONNECTED && nh.connected()){    // connected to Wifi and roscore
        chatter.publish(&pose_msg);
         digitalWrite(LED1_PIN, false);
         //digitalWrite(LED2_PIN, true);
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
             //digitalWrite(LED2_PIN, true);
         }
         Serial.println("");
     }
     else if (!nh.connected()){                              // only not connected to roscore
         Serial.println("DISCONNECTED FROM ROS");
          nh.advertise(chatter);
         timeout = 0;
         while (!nh.connected() && timeout <= 10){
             timeout++;
             delay(250);
             Serial.print(".");
             nh.spinOnce();

             lightTog = !lightTog;
             digitalWrite(LED1_PIN, lightTog);               // faster blink
             //digitalWrite(LED2_PIN, true);
         }
         Serial.println("");
     }

    /* #endregion */
    /* #region ------------------------------------------------BEHAVIOR----------------------------------------------------------------------*/

    // Robot.decision(state); // decides the overall state of the robot (wandering, tracking, etc.)

    // Robot.getOdom(left, right);
    Robot.getOdom();

    if(Robot.scentFilter(scent)){
        state = "FOUND_FOOD";
    }
    bool reachedGoal = Robot.reachedGoal();
    if(reachedGoal == true){
        Robot.driveWheel('R',0);
        Robot.driveWheel('L',0);
        delay(1000);
        Robot.decision = 0;
        Robot.nextStep(state); // decides how to decide desired location
    }
    else{
    
        // Robot.locateObstacle(Robot.viewDist);
        vels = Robot.controlLaw();
        int vel = vels.x;
        int ang_vel = vels.y;
        
        Serial.print("Total Vels: ");
        Serial.print(vel); Serial.print("\t");
        Serial.print(ang_vel); Serial.print("\t");

        Robot.moveRobot(vel, ang_vel);
    }

    // timer++;
    // if(timer>0 && timer<=50){
    //     Serial.print("forward");  Serial.println("\t");
    // Robot.driveWheel('R',200);
    // Robot.driveWheel('L',200);
    // }
    // else if(timer>50 && timer<=100){
    //     Serial.print("left");  Serial.println("\t");
    //     Robot.driveWheel('R',200);
    //     Robot.driveWheel('L',-200);
    // }
    // else if(timer>100 && timer<=150){
    //     Serial.print("backward");  Serial.println("\t");
    //     Robot.driveWheel('R',-200);
    //     Robot.driveWheel('L',200);
    // }
    // else if(timer>150 && timer<=200){
    //     Serial.print("right");  Serial.println("\t");
    //     Robot.driveWheel('R',-200);
    //     Robot.driveWheel('L',-200);
    // }
    // else if(timer>200){ 
    //     Serial.println("");
    //     timer = 0;
    // }

    /* #endregion */
    /* #region -------------------------------------------------EXTRAS-----------------------------------------------------------------------*/


    Serial.print("Current: ");
    Serial.print(Robot.currentLoc.Pos.x); Serial.print("\t");
    Serial.print(Robot.currentLoc.Pos.y); Serial.print("\t");
    Serial.print(Robot.currentLoc.yaw); Serial.print("\t");
    Serial.print("Desired: ");
    Serial.print(Robot.desiredLoc.Pos.x); Serial.print("\t");
    Serial.print(Robot.desiredLoc.Pos.y); Serial.print("\t");
    Serial.print(Robot.desiredLoc.yaw); Serial.print("\t");
    // Serial.print(sizeof(Robot.world.column[0])); Serial.print("\t");
    // Serial.print(state); Serial.print("\t");    
    // Serial.print(rightVel); Serial.print("\t");
    // Serial.print(leftVel); Serial.print("\t");
    // Serial.print(Robot.viewDist); Serial.print("\t");
    // Serial.print(scent); Serial .print("\t");
    // Serial.print(right); Serial.print("\t"); 
    // Serial.print(left); Serial.print("\t");
    // Serial.print(serverPort); Serial.print("\t");
    // Serial.print(double(Robot.squareWidth)); Serial.print("\t");

    Serial.println("");   
    
    
    
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
    for(int i = 0; i < 5; i++){
        if(Robot.currentPher[i] > 0){
            Robot.currentPher[i]-=1;
        }
    }
    // Loop exproximativly at 10Hz
    delay(Robot.loopLength);
    // nh.spinOnce();
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
        Serial.println(Robot.name);
    }
    else if (cmd[5] == 'r' && cmd[6] == 's' && cmd[7] == 's' && cmd[8] == 'i')
    { // "rssi/"
        Serial.println(WiFi.RSSI());
    }
    else if (cmd[4] == 's' && cmd[5] == 't' && cmd[6] == 'a' && cmd[7] == 't' && cmd[8] == 'e')
    { // "state/"
        Serial.println(state);
    }
    else if (cmd[5] == 'p' && cmd[6] == 'o' && cmd[7] == 'r' && cmd[8] == 't')
    { // "port/"
        Serial.println(serverPort);
    }
    else if (cmd[5] == 'p' && cmd[6] == 'o' && cmd[7] == 'r' && cmd[8] == 't')
    { // "port/"
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

void read_encL(){ //maintain encoder tick left wheel counts
    Robot.time_nowL = millis(); 
    Robot.delta_t_L = Robot.time_nowL-Robot.time_previousL;
    Robot.time_previousL = Robot.time_nowL;
    Robot.v_left = (((2*PI*Robot.radius)/Robot.ticks_per_rev) / (Robot.delta_t_L)) * Robot.rSign; //linear velocity left wheel (mm/ms)
    // Serial.print("Left"); Serial.print("\t");
}

void read_encR(){ //maintain encoder tick right wheel counts
    Robot.time_nowR = millis(); 
    Robot.delta_t_R = Robot.time_nowR-Robot.time_previousR;
    Robot.time_previousR = Robot.time_nowR;
    Robot.v_right = (((2*PI*Robot.radius)/Robot.ticks_per_rev) / (Robot.delta_t_R)) * Robot.lSign; //linear velocity right wheel (mm/ms)
    // Serial.print("Right"); Serial.print("\t");
}


