#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>

struct Location{
    int x, y, z;        // position
    int er, ep, ey;       // euler
    int qx, qy, qz, qw; // quarternion
};

class ant{
    public:
        int id, velocity, width, height;
        bool active;
        struct Location loc;
        

        ant(){
            loc = { 0,0,0,
                    0,0,0,
                    0,0,0,0};

        }

        void getOdom(int left, int right){
            // right = velocity of right wheel, left = velocity of left wheel
            // talk about control in lecture



            euler_to_quarternion();
        }

        void setPheromone(bool active){
            // based on position, send to 'world' node
            // if(active = true){special phereomones indicating they've found food}

        }

        void randomSearch(void){
            setPheromone(false);
            // wander around, avoid hitting walls, avoid pheromones of other robots

        }

        void findHelp(void){
            setPheromone(true);
            // drive outward from food, setting 'active' pheromones along the way

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
        
            return(diff);
        }

        void euler_to_quarternion(void){    // converts euler in Location struct to quarternion in Location struct
            loc.qw = cos(loc.er) * cos(loc.ep) * cos(loc.qy) - sin(loc.er) * sin(loc.ep) * sin(loc.qy);
            loc.qx = sin(loc.er) * sin(loc.ep) * cos(loc.ey) + cos(loc.er) * cos(loc.ep) * sin(loc.ey);
            loc.qy = sin(loc.er) * cos(loc.ep) * cos(loc.ey) + cos(loc.er) * sin(loc.ep) * sin(loc.ey);
            loc.qz = cos(loc.er) * sin(loc.ep) * cos(loc.ey) - sin(loc.er) * cos(loc.ep) * sin(loc.ey);
        }

};

#endif



// For passing structs into functions
// void data(Location *loc) {
// void loop(){
//     int data(&samples);
// }