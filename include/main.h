#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>
#include <String.h>

struct Location{
    int x, y, z;            // position
    int er, ep, ey;         // euler
    int qx, qy, qz, qw;     // quarternion
};

class ant{

    public:
        const int width=8, height=8;    //TODO: make real values
        int id, velocity;
        bool active;
        struct Location loc;
        String world[100][100];

        ant(){
            loc = { 0,0,0,
                    0,0,0,
                    0,0,0,0};
        }

        // constexpr uint32_t hash(const char* data, size_t const size) noexcept{  //for hashing strings to be used in switch
        //     uint32_t hash = 5381;

        //     for(const char *c = data; c < data + size; ++c)
        //         hash = ((hash << 5) + hash) + (unsigned char) *c;

        //     return hash;
        // } 

        void decision(String state){                                                                 
            // switch(hash(state)){
            // case hash("one") : // do something
            // case hash("two") : // do something
            // }
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
        
            int threshold = 20;
            if(diff >= threshold){      // TODO may want to seperate this into its own function...
                return(true);
            }
            else if(diff < threshold){
                return(false);
            }

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