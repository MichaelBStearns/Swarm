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

struct Coords{
    int x, y;
};

struct Location{
    Coords Pos;             // position
    Coords Grid;            // grid position
    int roll, pitch, yaw;   // euler
    int qx, qy, qz, qw;     // quarternion
};

class ant{

    public:
        const int   width=94, height=9, diameter=65,    // mm (wheel center to center, IR to ground, wheel)
                    l=width/2, radius=diameter/2, 
                    gridWidth = 200, gridHeight = gridWidth, 
                    squareWidth = sizeof(world)/ gridWidth, squareHeight = squareWidth,
                    k_rho = 2, k_alpha = 15, k_beta = -8,			// control coefficients
										rpm_convert = 6000, ticks_per_rev 20;    
        int id, velocity, rightOld=-1, leftOld=-1,
            vel = 0, ang_vel = 0,
            d_x, d_y, theta,
						countR, countL = 0; 
	
				double 	theta, phi = 0.0,
								time_nowL, time_nowR, time_previousL, time_previousR, delta_t_R, delta_t_L = 0.0,
								omega_left, omega_right = 0.0,
								v_center, v_right, v_left = 0.0,	
								revL, revR = 0.0,
								x_prime, x, y_prime, y, theta_prime = 0.0;
									
									
        bool active;    //pheromones
        struct Location currentLoc, desiredLoc;
        uint8_t currentPher[5], prevPher[5]; 
        swarm_msgs::Grid world;   // same as Grid being sent

        ant(){
            currentLoc = {  0,0,
                            0,0,
                            0,0,0,
                            0,0,0,0};
        }

        void decision(String state){
            if(state == "ROAM"){    // no food or pheromones found

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

        void nextStep(String state){
            if(state == "ROAM"){    // no food or pheromones found
                randomSearch();
            }
            else if(state == "FOUND_FOOD"){     // located goal, see if anyone else has been there, if not: get help, if so: wait
                
            }
            else if(state == "GET_HELP"){   // go back to start & leave pheromones
                
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
            // talked about control in lecture

            v_center = (right + left)/2; //linear velocity of robot center
            phi = (right - left)/wheelbase; //calculates yaw
            theta_prime = theta + phi; 
            if (theta_prime >= 2*PI){  //adjust theta_prime so that it is between 0 and 2pi
                while(theta_prime > 2*PI){
                    theta_prime = theta_prime - 2*PI;
                }
            } else if (theta_prime < 0){
                while(theta_prime < 0){
                    theta_prime = theta_prime + 2*PI;
                }
            }
            x_prime = x + v_center*cos(theta); //new x pos from old x
            y_prime = y + v_center*sin(theta); //new y pos from old y
            theta = theta_prime; //update pose angle

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
                analogWrite(motorL_for, pwm);
                analogWrite(motorL_back, 0);
            }
            else if((wheel == 'l' || wheel == 'L') && (dir == 'b' || dir == 'B')){
                analogWrite(motorL_for, 0);
                analogWrite(motorL_back, pwm);
            }
        }

        void setPheromone(bool active){
            // based on position, send to 'world' node
            // if(active = true){special phereomones indicating they've found food}
        }
        
        
        void randomSearch(void){
            // wander around, avoid hitting walls
            setPheromone(false);
            
            int choice = random(1,100);
            uint8_t weights[] = {0, 10, 20, 31, 20, 10, 4, 4, 1}; //toal 100
            uint8_t weightsums[9] = {0};

            weightsums[0] = weights[0];
            for(uint8_t i=1; i < sizeof(weights); i++){
                weightsums[i] = weights[i] + weightsums[i-1];
            }
            if(weightsums[8] != 100){
                try{throw "WEIGHTS DON'T TOTAL 1";} catch(int E){Serial.print("AN EXCEPTION WAS THROWN: "); Serial.print(E);}     // throw exception 
            }

        /*  -------------
            | 4 | 3 | 2 |
            -------------
            | 5 | C | 1 |
            -------------
            | 6 | 7 | 8 |
            -------------   */

            if(choice >= 0 && choice <= weightsums[0]){                     // C
                desiredLoc.Grid.x = currentLoc.Grid.x + 0;
                desiredLoc.Grid.y = currentLoc.Grid.y + 0;
                }
            else if(choice >= weightsums[0] && choice <= weightsums[1]){    // 1
                desiredLoc.Grid.x = currentLoc.Grid.x + 1;
                desiredLoc.Grid.y = currentLoc.Grid.y + 0;
                }
            else if(choice >= weightsums[1] && choice <= weightsums[2]){    // 2
                desiredLoc.Grid.x = currentLoc.Grid.x + 1;
                desiredLoc.Grid.y = currentLoc.Grid.y + 1;
                }
            else if(choice >= weightsums[2] && choice <= weightsums[3]){    // 3
                desiredLoc.Grid.x = currentLoc.Grid.x + 0;
                desiredLoc.Grid.y = currentLoc.Grid.y + 1;
                }
            else if(choice >= weightsums[3] && choice <= weightsums[4]){    // 4
                desiredLoc.Grid.x = currentLoc.Grid.x - 1;
                desiredLoc.Grid.y = currentLoc.Grid.y + 1;
                }
            else if(choice >= weightsums[4] && choice <= weightsums[5]){    // 5
                desiredLoc.Grid.x = currentLoc.Grid.x - 1;
                desiredLoc.Grid.y = currentLoc.Grid.y + 0;
                }
            else if(choice >= weightsums[5] && choice <= weightsums[6]){    // 6
                desiredLoc.Grid.x = currentLoc.Grid.x - 1;
                desiredLoc.Grid.y = currentLoc.Grid.y - 1;
                }
            else if(choice >= weightsums[6] && choice <= weightsums[7]){    // 7
                desiredLoc.Grid.x = currentLoc.Grid.x - 1;
                desiredLoc.Grid.y = currentLoc.Grid.y - 0;
                }
            else if(choice >= weightsums[7] && choice <= weightsums[8]){    // 8
                desiredLoc.Grid.x = currentLoc.Grid.x + 1;
                desiredLoc.Grid.y = currentLoc.Grid.y - 1;
                }
        }

        int * controlLaw(void){
            if(k_alpha + (5/3)*k_beta - (2/PI)*k_rho <= 0){
                try{throw "NO ROBUST STABILITY";} catch(int E){Serial.print("AN EXCEPTION WAS THROWN: "); Serial.print(E);}     // throw exception 
            }
            int threshold = 0.01;
            
            d_x = desiredLoc.Pos.x - currentLoc.Pos.x, 
            d_y = desiredLoc.Pos.y - currentLoc.Pos.y,
            theta = currentLoc.yaw; 

            int rho = sqrt(d_x^(2)+d_y^(2));
            int alpha = -theta + atan2(d_y, d_x);
            int beta = -theta - alpha;      
            // int beta = -alpha;      // make just -alpha so doesnt have to correct orientation?
            
            vel = k_rho*rho;
            ang_vel = k_alpha*alpha+k_beta*beta;

            if ((abs(d_x)+abs(d_y)+abs(theta)) > threshold){
                
            }




            int output[] = {vel, ang_vel};
            return output;
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
        
            // Serial.print(diff); Serial.println("\t");

            int threshold = 200;
            if(diff >= threshold){      // TODO may want to seperate this into its own function...
                return(true);
            }
            else{ // if(diff < threshold)
                return(false);
            }

        }

        void euler_to_quarternion(void){    // converts euler in Location struct to quarternion in Location struct
            currentLoc.qw = cos(currentLoc.roll) * cos(currentLoc.pitch) * cos(currentLoc.qy) - sin(currentLoc.roll) * sin(currentLoc.pitch) * sin(currentLoc.qy);
            currentLoc.qx = sin(currentLoc.roll) * sin(currentLoc.pitch) * cos(currentLoc.yaw) + cos(currentLoc.roll) * cos(currentLoc.pitch) * sin(currentLoc.yaw);
            currentLoc.qy = sin(currentLoc.roll) * cos(currentLoc.pitch) * cos(currentLoc.yaw) + cos(currentLoc.roll) * sin(currentLoc.pitch) * sin(currentLoc.yaw);
            currentLoc.qz = cos(currentLoc.roll) * sin(currentLoc.pitch) * cos(currentLoc.yaw) - sin(currentLoc.roll) * cos(currentLoc.pitch) * sin(currentLoc.yaw);
        }

        void pos_to_Grid(void){     //calculates which grid square the robot is in based on position
            currentLoc.Grid.x = currentLoc.Pos.x / squareWidth;
            currentLoc.Grid.y = currentLoc.Pos.y / squareHeight;
        }

        void grid_to_Pos(void){     //calculates which grid square the robot is in based on position
            currentLoc.Pos.x = currentLoc.Grid.x * squareWidth;
            currentLoc.Pos.y = currentLoc.Grid.y * squareHeight;
        }

};

#endif

void read_encR(){ //maintain encoder tick right wheel counts
    countR = countR + 1;
    time_nowR = millis(); 
    delta_t_R = time_nowR-time_previousR;
    time_previousR = time_nowR;
    v_right = countR /(rpm_convert * delta_t_R); //linear velocity left wheel
    countL = 0;
}

void read_encL(){ //maintain encoder tick left wheel counts
    countL = countL + 1;
    time_nowL = millis(); 
    delta_t_L = time_nowL-time_previousL;
    time_previousL = time_nowL;
    v_left = countL /(rpm_convert * delta_t_L); //linear velocity left wheel
    countL = 0;
}

// For passing structs into functions
// void data(Location *currentLoc) {
// void loop(){
//     int data(&samples);
// }