#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>
#include <String.h>
#include <swarm_msgs/Grid.h>
// #include <main.cpp>

// Motor Control
#define motorR_back D6
#define motorR_for D7 
#define motorL_for D8
#define motorL_back D9

struct Location{
    int x, y, z;            // position
    int gx, gy;             // grid position
    int er, ep, ey;         // euler
    int qx, qy, qz, qw;     // quarternion
};

class ant{

    public:
        const int width=8, height=8;    //TODO: measure values
        int id, velocity, rightOld=-1, leftOld=-1;
        bool active;    //pheromones
        struct Location currentLoc, desiredLoc;
        uint8_t currentPher[5], prevPher[5];
        swarm_msgs::Grid world;   // same as Grid being sent

        ant(){
            currentLoc = { 0,0,0,
                    0,0,0,
                    0,0,0,0};
        }

        void decision(String state){
            if(state == "ROAM"){    // no food or pheromones found
                randomSearch();
            }
            else if(state == "FOUND_FOOD"){     // located goal, see if anyone else has been there, if not: get help, if so: wait
                //
            }
            else if(state == "GET_HELP"){   // go back to start & leave pheromones
                findHelp();
            }
            else if(state == "FOLLOW_TRAIL"){   //found pheromones, follow it back to food
                //
            }
            else if(state == ""){

            }
            else{   // if no state found
                try{throw "NO DECISION STATE FOUND";} catch(int E){Serial.print("AN EXCEPTION WAS THROWN: "); Serial.print(E);}     // throw exception 
            }
        }

        int * Drive(int alpha, int theta, int beta){
            int* right, left;
        

            return right;
        }

        void getOdom(int left, int right){
            // right = velocity of right wheel, left = velocity of left wheel
            // talked about control in lecture

            euler_to_quarternion();
        }
        
        void setOdom(int left, int right){
            
        }

        void driveWheel(char wheel, char dir, int pwm){
            if((wheel == 'r' || wheel == 'R') && (dir == 'f' || dir == 'F')){
                analogWrite(motorR_for, pwm);
                analogWrite(motorR_back, 0);
            }
            else if((wheel == 'r' || wheel == 'R') && (dir == 'b' || dir == 'B')){
                analogWrite(motorR_for, 0);
                analogWrite(motorR_back, pwm);
            }
            else if((wheel == 'l' || wheel == 'L') && (dir == 'f' || dir == 'F')){
                analogWrite(motorR_for, pwm);
                analogWrite(motorR_back, 0);
            }
            else if((wheel == 'l' || wheel == 'L') && (dir == 'b' || dir == 'B')){
                analogWrite(motorR_for, 0);
                analogWrite(motorR_back, pwm);
            }


        }

        void setPheromone(bool active){
            // based on position, send to 'world' node
            // if(active = true){special phereomones indicating they've found food}

        }

        void randomSearch(void){
            setPheromone(false);
            // wander around, avoid hitting walls

        }

        void findHelp(void){
            setPheromone(true);
            // drive back to start, setting 'active' pheromones along the way

        }

        double ReadIR(int sensor){  //sends a signal with the IR sensor then detects how long it takes to come back
            pinMode(sensor, OUTPUT );
            digitalWrite(sensor, HIGH);
            delayMicroseconds(10);
            pinMode(sensor, INPUT);
            long reading = micros();

            //time how long the input is HIGH, but quit after 3ms as nothing happens after that
            while (digitalRead(sensor) == HIGH && micros() - reading < 3000);
            int diff = micros() - reading;
        
            Serial.print(diff); Serial.print("\t");

            int threshold = 200;
            if(diff >= threshold){      // TODO may want to seperate this into its own function...
                return(true);
            }
            else{ // if(diff < threshold)
                return(false);
            }

        }

        void euler_to_quarternion(void){    // converts euler in Location struct to quarternion in Location struct
            currentLoc.qw = cos(currentLoc.er) * cos(currentLoc.ep) * cos(currentLoc.qy) - sin(currentLoc.er) * sin(currentLoc.ep) * sin(currentLoc.qy);
            currentLoc.qx = sin(currentLoc.er) * sin(currentLoc.ep) * cos(currentLoc.ey) + cos(currentLoc.er) * cos(currentLoc.ep) * sin(currentLoc.ey);
            currentLoc.qy = sin(currentLoc.er) * cos(currentLoc.ep) * cos(currentLoc.ey) + cos(currentLoc.er) * sin(currentLoc.ep) * sin(currentLoc.ey);
            currentLoc.qz = cos(currentLoc.er) * sin(currentLoc.ep) * cos(currentLoc.ey) - sin(currentLoc.er) * cos(currentLoc.ep) * sin(currentLoc.ey);
        }

};

#endif



// For passing structs into functions
// void data(Location *currentLoc) {
// void loop(){
//     int data(&samples);
// }