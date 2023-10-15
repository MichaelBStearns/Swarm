/* rosserial Publisher Example
 * Prints "hello world!"
 * This intend to connect to a Wifi Access Point
 * and a rosserial socket server.
 * You can launch the rosserial socket server with
 * roslaunch rosserial_server socket.launch
 * The default port is 11411
 *
 */

/*                        NodeMCU
                    /-------------------\
USED (V meter) - A0 | ADC0       GPIO16 | D0 - USED (LED)
                    | RES         GPIO5 | D1 - USED (SCL-Lidar)        
                    | RES         GPIO4 | D2 - USED (SDA-Lidar)    
  USED ()     - SD3 | GPIO10      GPIO0 | D3 - UNUSED ()  MUST BE LOW TO FLASH - HIGH OR FLOATING FOR BOOT
                    | GPIO9       GPIO2 | D4 - UNUSED ()    
                    | MOSI         3.3V | 
                    | CS            GND | 
                    | MISO       GPIO14 | D5 - USED (Right Wheel Servo)
                    | SCLK       GPIO12 | D6 - USED (Right Wheel Pot)
                    | GND        GPIO13 | D7 - USED (Left Wheel Pot)  
                    | 3.3V       GPIO15 | D8 - USED (Left Wheel Servo)  
                    | EN          GPIO3 | RX - UNUSED 
                    | RST         GPIO1 | TX - UNUSED
                    | GND           GND |    
                    | Vin          3.3V |    
                    \--------||||-------/
                             ||||
*/

/* Links:
    NodeMCU pins:
        https://randomnerdtutorials.com/esp8266-pinout-reference-gpios/
    Sensors:
        QRE1113 IR sensor - https://www.sparkfun.com/products/9454 
        VL53L0X Lidar sensor - https://www.pololu.com/product/2490/specs
        Feedback 360 Degree Servo - https://www.adafruit.com/product/3614
*/

#include <Arduino.h>
#include <main.h>
#include "Wire.h"
#include <ESP8266WiFi.h>

#define ROSSERIAL_ARDUINO_TCP
#include <ros.h>
// #include <std_msgs/String.h>
#include <swarm_msgs/Plus.h>    //Header, Pose, bool, string, float32[5]
#include <swarm_msgs/Grid.h>  

#include "Wire.h"
#include <VL53L0X.h>
#include <string>
#include <PID_v1.h>
#include <Servo.h>

#define CONNECT         // to connect to wifi
#define ROS             // to connect to ROS

#define LED1_PIN D0     // D0 and D4 are the built in LEDs        
// #define LED2_PIN D4
#define RightServoWrite D5
#define RightServoRead D6
#define LeftServoRead D7
#define LeftServoWrite D8

#define FOOD_PIN D1     //?

// #define LED1_PIN D0     

// Enable reading supply voltage
ADC_MODE(ADC_VCC);

#ifdef CONNECT
    // Network info
    const char *ssid = WIFI_SSID;
    const char *password = WIFI_PASS;
    // Set the rosserial socket server IP address
    IPAddress server(10, 5, 6, 3);
#endif

#ifdef ROS
    // Set the rosserial socket server port
    const uint16_t serverPort = BUILD_ENV_PORT;

    // ROS Node for each Robot
    ros::NodeHandle nh;
        // Make a publisher
        swarm_msgs::Plus pose_msg;
        ros::Publisher chatter("/Pheromones_Write", &pose_msg);
        // Make a subscriber
        void sub_msg(const swarm_msgs::Grid& msg);
        ros::Subscriber<swarm_msgs::Grid> smell("/Pheromones_Read", &sub_msg);
#endif

// Robot class
ant Robot;
// Drive servos class
Servo RightWheel;
Servo LeftWheel;
// Lidar sensor class
VL53L0X sensor;

// float scent, right, left, Hz, currentTime, Vcc;
// bool printTog = true;
// int *wheels, timer = 0;
// byte address = 41; // address 0x29
// String state = "ROAM";  // "ROAM", "FOUND_FOOD", "GET_HELP", "FOLLOW_TRAIL"
// char cmd[10];
// struct Coords vels;

// Initialize Variables
bool lightTog = false;
int speed = 90;  //0 = full CW, 90 = still, 180 = full CCW
int input = 0, full = 0, sign = 1;  //for serial user input
int timeout = 0, setupTime = 10;
int scent;
double riseL, riseR, fallL, fallR, diffL, diffR, LeftWheelPosTemp, RightWheelPosTemp;

// double Setpoint, Input, Output;
// double Kp=2, Ki=5, Kd=1;
// PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// Initialize Functions
void IRAM_ATTR read_left_servo(void);     //left encoder interrupt //IRAM_ATTR: ROM, ICACHE_RAM_ATTR: RAM
void IRAM_ATTR read_right_servo(void);     //left encoder interrupt //IRAM_ATTR: ROM, ICACHE_RAM_ATTR: RAM
void terminal(void);

void setup() {
    // Use ESP8266 serial to monitor the process
    Serial.begin(115200);
    Serial.print("Hello World! I am "); Serial.println(Robot.name);

    RightWheel.attach(RightServoWrite);
    LeftWheel.attach(LeftServoWrite);

    RightWheel.write(90);    //make servos stay still
    LeftWheel.write(90);    //make servos stay still
    
    // pinMode(RightServoWrite, OUTPUT);
    pinMode(LeftServoRead, INPUT);
    pinMode(RightServoRead, INPUT);
    // pinMode(LeftServoWrite, OUTPUT);
    // pinMode(RightServoWrite, OUTPUT);

    attachInterrupt(digitalPinToInterrupt(LeftServoRead), read_left_servo, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RightServoRead), read_right_servo, CHANGE);

    //initialize the variables we're linked to
    // Setpoint = 0;

    //turn the PID on
    // myPID.SetMode(AUTOMATIC);

    // join 12C bus and set up Lidar
    Wire.begin(D1, D2); // SDA, SCL     //4, 5 ?
    sensor.init();
    sensor.setTimeout(500);
    sensor.startContinuous();

    #ifdef CONNECT  //Wi-fi
        Serial.print("Connecting to "); Serial.println(ssid); Serial.print("on port "); Serial.println(serverPort);
        // Connect the ESP8266 the the wifi AP
        WiFi.begin(ssid, password);
        while (WiFi.status() != WL_CONNECTED);// && timeout <= setupTime)
        {
            timeout++;
            delay(500);
            Serial.print(".");

            lightTog = !lightTog;
            digitalWrite(LED1_PIN, lightTog);
            // digitalWrite(LED2_PIN, true);
        }
        Serial.println("");
        Serial.println("WiFi connected");
        Serial.print("IP address: ");
        Serial.println(WiFi.localIP());

        // Set the connection to rosserial socket server
        nh.getHardware()->setConnection(server, serverPort);
        nh.initNode();
        // Start to be polite
        nh.advertise(chatter);
        // Start asking questions
        nh.subscribe(smell);
    #endif
    #ifndef CONNECT
        delay(1000);    //give interrupts plenty of time to get first readings
    #endif

 /* #region - Preestablish servo positions to begin calculations */
    Robot.LeftWheelPos[0] = LeftWheelPosTemp;
    Robot.LeftWheelPos[1] = LeftWheelPosTemp;
    Robot.LeftWheelPos[2] = LeftWheelPosTemp;
    Robot.LeftWheelPos[3] = LeftWheelPosTemp;
    Robot.LeftWheelPos[4] = LeftWheelPosTemp;
    Robot.RightWheelPos[0] = RightWheelPosTemp;
    Robot.RightWheelPos[1] = RightWheelPosTemp;
    Robot.RightWheelPos[2] = RightWheelPosTemp;
    Robot.RightWheelPos[3] = RightWheelPosTemp;
    Robot.RightWheelPos[4] = RightWheelPosTemp;
/* #endregion */   

    Serial.println("start");
    
    // Wait until user is ready
    // Serial.print("Press any key to begin");
    // while (Serial.available() != 1);     //wait until any key is pressed
    // Serial.read();   //clear serial buffer

}

void loop() {

    if (Serial.available() != 0){       //Serial input
        void terminal();
    }


/* #region ------------------------------------------------DRIVING-----------------------------------------------------------------------*/
    
    Robot.getOdom();
    Robot.controlLaw();
    // Robot.moveRobot(Robot.wheelControl.x, Robot.wheelControl.y);
    RightWheel.write(speed);
    LeftWheel.write(speed); 

/* #endregion */
/* #region ---------------------------------------------ROS-CONNECTION-------------------------------------------------------------------*/

/* #endregion */
/* #region ---------------------------------------------SENSOR--INPUTS-------------------------------------------------------------------*/
    if (sensor.timeoutOccurred())   // if Lidar sensor times out
    {try{throw "LIDAR TIMEOUT";} catch (int E){Serial.print("AN EXCEPTION WAS THROWN: ");Serial.print(E);}} // throw exception

    Robot.viewDist = sensor.readRangeContinuousMillimeters(); // gives Lidar distance in mm
    scent = Robot.readIR(FOOD_PIN);                     // returns true if food found, returns false if not
    if(scent > Robot.IRThreshold){
        //food
    }
    else{
        //no food
    }

    Robot.VCC = ESP.getVcc()/1000.00;   //read input voltage
/* #endregion */
/* #region ------------------------------------------------BEHAVIOR----------------------------------------------------------------------*/

/* #endregion */
/* #region ----------------------------------------------CALCULATIONS--------------------------------------------------------------------*/

/* #endregion */
/* #region -------------------------------------------------EXTRAS-----------------------------------------------------------------------*/

/* #endregion */
/* #region ---------------------------------------------------END------------------------------------------------------------------------*/

    // nh.spinOnce();
    // Loop at 10Hz
    delay(10);
/* #endregion */
}


void read_left_servo(){ //maintain encoder tick left wheel counts
    if(digitalRead(LeftServoRead) == HIGH){  //rising edge
        riseL = micros();
    }
    else{  //falling edge
        fallL = micros();
        diffL = fallL - riseL;
        diffL = map(diffL, 30, 1060, 0, 1100);  //pot doesnt quite go to 0 or max
        LeftWheelPosTemp = diffL / 1100;  //1.1 ms period
        Robot.LeftWheelPos[0] = LeftWheelPosTemp * Robot.ServoResolution;
    }  
}

void read_right_servo(){ //maintain encoder tick right wheel counts
    if(digitalRead(RightServoRead) == HIGH){  //rising edge
        riseR = micros();
    }
    else{  //falling edge
        fallR = micros();
        diffR = fallR - riseR;
        diffR = map(diffR, 30, 1055, 1, 1100);  //pot doesnt quite go to 0 or max
        RightWheelPosTemp = diffR / 1100;  //1.1 ms period
        Robot.RightWheelPos[0] = RightWheelPosTemp * Robot.ServoResolution;
    }  
}

void sub_msg(const swarm_msgs::Grid& msg){  //reads pheremones in current square from ROS and assigns values to array variables
    for(int i = 0; i < 5;i++){      
        Robot.currentPher[i] = msg.column[(int)Robot.currentLoc.Grid.x].row[(int)Robot.currentLoc.Grid.y].pheromones[i];
    }
}

void terminal(){    //process user serial inputs
    input = Serial.read();
    if(input == 10){    // Enter
        if(full == -1000){  //reset position and yaw w/ "r"
            Robot.currentLoc.Pos.x = 0;
            Robot.currentLoc.Pos.y = 0;
            Robot.theta = 0;
        }
        else if(full != 0){
            speed = full * sign;
            Serial.print("\t");
            Serial.println(speed);
            sign = 1;
            full = 0;
        }
        else{
            Serial.print(Robot.LeftWheelPos[0]); Serial.print("\t");
            Serial.println(Robot.RightWheelPos[0]); Serial.print("\t");
        }
    }
    else if((input == 45)){     // -
        sign = -1; 
        // Serial.print("-");
    }
    else if((input == 114)){    // r
        full = -1000; 
        Serial.print("r");
    }
    else if((input >= 48) && (input <= 57)){    // 0-9
        full = (full * 10) + (input - 48);
        
        Serial.print((input - 48));
    }
}