#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>
#include <String.h>
#include <swarm_msgs/Grid.h>
// #include <main.cpp>

#define motorR_back D3          // Motor Control
#define motorR_for D4
#define motorL_back D8
#define motorL_for D6

struct Coords{
    double x, y;
};

struct Location{
    Coords Pos;                 // position (mm)
    Coords Grid;                // grid position
    double roll, pitch, yaw;    // euler
    double qx, qy, qz, qw;      // quarternion
};

class ant{
    public:
        const int           width=94, height=9, diameter=65,  // mm (wheel center to center, IR to ground, wheel)
                            ServoResolution = 360,
                            gridWidth = 2000, gridHeight = gridWidth,
					        k_rho = 2, k_alpha = 30, k_beta = -5,		// control coefficients
                            viewThreshold = 3000, loopLength = 100, goalStopThreshold = 10;
        const float         l=width/2, radius=diameter/2, circum=2*PI*radius,
                            squaresinGrid = sizeof(world.column)/sizeof(world.column[0]),
                            squareWidth = gridWidth / squaresinGrid, squareHeight = squareWidth;
        const String        name = BUILD_ENV_NAME;

        struct Location     currentLoc, desiredLoc;
        int                 id, rightOld=-1, leftOld=-1,
                            LeftWheelPosInt, LeftWheelPosOldInt, RightWheelPosInt, RightWheelPosOldInt,
                            vel = 0, ang_vel = 0, viewDist, decision = 0, 
                            scents[5] = {0,0,0,0,0},
                            IRThreshold;
		    float 	            velocity, distance,
                            theta = 0, phi = 0, phiDot = 0,  //angular velocity              //radians
                            previousTimeLeft, previousTimeRight, timeDeltaRight = 0, timeDeltaLeft = 0,
                            omega_left = 0, omega_right = 0, v_center = 0, velocityRight = 0, velocityLeft = 0, revL = 0, revR = 0,
                            rSign = 1, lSign = 1,
                            d_x, d_y, d_theta,
                            LeftWheelPos[5] = {0,0,0,0,0}, RightWheelPos[5] = {0,0,0,0,0}, distanceRight, distanceLeft,
                            VCC;	                      

        bool                active = false;    //pheromones
        uint8_t             currentPher[5], prevPher[5];
        swarm_msgs::Grid    world;   // same as Grid being sent (no pheromones in obstacles so 'N' doesnt overwrite any data)
        Coords              path[30], wheelControl;   //store last locations to know where not to go

        ant(){
            currentLoc = {  0,0, 
                            0,0,
                            0,0,0,
                            0,0,0,0};
            currentLoc.Pos.x = 0; //double(squareWidth);
            currentLoc.Pos.y = 0; //double(squareWidth);

            desiredLoc = {  0,0,
                            0,0,
                            0,0,0,
                            0,0,0,0};
            desiredLoc.Pos.x = 0; //double(squareWidth);
            desiredLoc.Pos.y = 0; //double(squareWidth);
        }

        void getOdom(void){     // gets currentLoc info from encoders   
            //filter out noise spikes (if one value is very differnt, smooth it out)
            if(abs(LeftWheelPos[2] - LeftWheelPos[0]) <= 1.5){
                LeftWheelPos[1] = LeftWheelPos[2];
            }
            if(abs(RightWheelPos[2] - RightWheelPos[0]) <= 1.5){
                RightWheelPos[1] = RightWheelPos[2];
            }

            // encoder rollover correction
            if(abs(RightWheelPos[1]-RightWheelPos[2]) >= (ServoResolution/2)){
                if(RightWheelPos[1] > RightWheelPos[2]){
                    distanceRight = RightWheelPos[2]-(RightWheelPos[1]-ServoResolution);
                }
                else{
                    distanceRight = (RightWheelPos[2]-ServoResolution)-RightWheelPos[1];
                }
            }
            else{
                distanceRight = -(RightWheelPos[2]-RightWheelPos[1]);
            }
            if(abs(LeftWheelPos[1]-LeftWheelPos[2]) >= (ServoResolution/2)){
                if(LeftWheelPos[1] > LeftWheelPos[2]){
                    distanceLeft = LeftWheelPos[2]-(LeftWheelPos[1]-ServoResolution);
                }
                else{
                    distanceLeft = (LeftWheelPos[2]-ServoResolution)-LeftWheelPos[1];
                }
            }
            else{
                distanceLeft = LeftWheelPos[2]-LeftWheelPos[1];
            }

        }

        bool avoidSpace(int x, int y){    // make sure robot doesn't want to go out of bounds or into a wall
            //Serial.println(world.column[x].row[y].pheromones[0]);
            return (world.column[x].row[y].pheromones[0] == 'N' || x >= squaresinGrid || y >= squaresinGrid|| x <= 0 || y <= 0 || world.column[x].row[y].pheromones[0] > 80 ) ? true : false; 
        }

        int randomSearch(void){
            // wander around, avoid hitting walls
            
            // Serial.print("Pos Left: "); //Serial.print("\t");
            // Serial.print(LeftWheelPos[0]); Serial.print("\t");          
            // Serial.print(LeftWheelPos[1]); Serial.print("\t");          
            // Serial.print(LeftWheelPos[2]); Serial.print("\t");          
            // Serial.print(LeftWheelPos[3]); Serial.print("\t");          
            // Serial.print(LeftWheelPos[4]); Serial.print("\t"); 

            // calculate difference since last measurement
            // distanceLeft = (LeftWheelPos[2] - LeftWheelPos[1]);
            // distanceRight = -(RightWheelPos[2] - RightWheelPos[1]);

            //rotate array through at end of function

            // calculate arc length (degree -> distance)
            distanceLeft = (distanceLeft/360)*circum;
            distanceRight = (distanceRight/360)*circum;

            float d_center = (distanceRight + distanceLeft)/2; 
            
            //angular velocity of robot center
            // phiDot = 2*PI*(mm_velocityRight - mm_velocityLeft)/(width);
            // phi = phiDot / (timeDeltaLeft+timeDeltaRight);
            phi = (distanceRight - distanceLeft) / (width);
            
            Serial.print(" Distance Right: "); Serial.print(distanceRight); Serial.print("\t");
            Serial.print("Distance Left: "); Serial.print(distanceLeft); Serial.print("\t");
            Serial.print("Angle: "); Serial.print(phi); Serial.print("\t");

            theta = theta + phi; // yaw = theta

            //make sure the angle stays within 2 pi
            while(theta > 2*PI){
                theta = theta - 2*PI;
            }
            while(theta < 0){
                theta = theta + 2*PI;
            }
            currentLoc.yaw = theta;// * (180 / PI);
            
            Serial.print("Yaw: "); Serial.print(currentLoc.yaw); Serial.print("\t");

            Serial.print("Distance: "); //Serial.print("\t");
            Serial.print(d_center); Serial.print("\t");
            
            currentLoc.Pos.x = currentLoc.Pos.x + d_center * cos(currentLoc.yaw); //new x pos from old x
            currentLoc.Pos.y = currentLoc.Pos.y - d_center * sin(currentLoc.yaw); //new y pos from old y


            Serial.print("New Location: ");// Serial.print("\t");
            Serial.print(currentLoc.Pos.x); Serial.print("\t");
            Serial.print(currentLoc.Pos.y); Serial.print("\t");

            timeDeltaLeft = 0;
            timeDeltaRight = 0;         
            
            LeftWheelPos[4] = LeftWheelPos[3];
            LeftWheelPos[3] = LeftWheelPos[2];
            LeftWheelPos[2] = LeftWheelPos[1];
            LeftWheelPos[1] = LeftWheelPos[0];

            RightWheelPos[4] = RightWheelPos[3];
            RightWheelPos[3] = RightWheelPos[2];
            RightWheelPos[2] = RightWheelPos[1];
            RightWheelPos[1] = RightWheelPos[0];

            Serial.println("");

            euler_to_quarternion(); // calculated quarternion fron existing euler
        }

        Coords controlLaw(void){    //the control law for the motors (like PID?)
            if(k_alpha + (5/3)*k_beta - (2/PI)*k_rho <= 0){     // if coefficients are not robustly stable
                try{throw "NO ROBUST STABILITY";} catch(int E){Serial.print("AN EXCEPTION WAS THROWN: "); Serial.print(E);}     // throw exception 
            }
            int threshold = 0.01;
            
            // Serial.print("Diffs: ");
            // Serial.print(d_x); Serial.print("\t");
            // Serial.print(d_y); Serial.print("\t");
            // Serial.print(d_theta); Serial.print("\t");

            d_x = (desiredLoc.Pos.x - currentLoc.Pos.x),   // difference between current and goal
            d_y = (desiredLoc.Pos.y - currentLoc.Pos.y), 
            d_theta = desiredLoc.yaw - currentLoc.yaw; 

            double rho = (sqrt(pow(d_x,2)+pow(d_y,2)));
            double alpha = -d_theta + atan2(d_y, d_x);
            // double beta = -d_theta - alpha;      
            int beta = -alpha;      // make just -alpha so doesnt have to correct orientation?
            
            // Serial.print("Coefs: ");
            // Serial.print(rho); Serial.print("\t");
            // Serial.print(alpha); Serial.print("\t");
            // Serial.print(beta); Serial.print("\t");

            vel = k_rho*rho/1;   // mm/s -> m/s         //2
            ang_vel = k_alpha*alpha+k_beta*beta;        //15    -8

            wheelControl.x = vel;
            wheelControl.y = ang_vel;
            return wheelControl;
        }

        void moveRobot(int vel, int ang_vel){
            int velocityRight, velocityLeft, rdrive, ldrive;
            velocityRight = vel + (ang_vel * radius);
            velocityLeft = vel - (ang_vel * radius);

            Serial.print("WriteVels: ");
            Serial.print(velocityRight); Serial.print("\t");
            Serial.print(velocityLeft); Serial.print("\t");

            if(velocityRight < 0){velocityRight = 0;}
            if(velocityLeft < 0){velocityLeft = 0;}
            if(velocityRight > 3000){velocityRight = 3000;}
            if(velocityLeft > 3000){velocityLeft = 3000;}

            Serial.print("WriteVels: ");
            Serial.print(velocityRight); Serial.print("\t");
            Serial.print(velocityLeft); Serial.print("\t");

            velocityRight = map(velocityRight, 0, 3000, 60, 100); //convert to PWM that makes the motor turn
            velocityLeft = map(velocityLeft, 0, 3000, 60, 100);

            driveWheel('R',velocityRight);
            driveWheel('L',velocityLeft);

            rSign = velocityRight/ abs(velocityRight);
            lSign = velocityLeft/ abs(velocityLeft);


            // while(velocityRight > 0 && v_right == 0){             //make sure wheel is turning if it watns to be
            //     velocityRight++;
            //     driveWheel('R',velocityRight);
            // }   
            // while(velocityLeft > 0 && v_left == 0){
            //     velocityLeft++;
            //     driveWheel('L',velocityLeft);           
            // }

            Serial.print("WriteVels: ");
            Serial.print(velocityRight); Serial.print("\t");
            Serial.print(velocityLeft); Serial.print("\t");
        }

        void driveWheel(char wheel, int pwm){
            if((wheel == 'r' || wheel == 'R') && (pwm >= 0)){
                analogWrite(motorR_for, abs(pwm));
                analogWrite(motorR_back, 0);
            }
            else if((wheel == 'r' || wheel == 'R') && (pwm < 0)){
                analogWrite(motorR_for, 0);
                analogWrite(motorR_back, abs(pwm));
            }
            else if((wheel == 'l' || wheel == 'L') && (pwm >= 0)){
                analogWrite(motorL_for, abs(pwm));
                analogWrite(motorL_back, 0);
            }
            else if((wheel == 'l' || wheel == 'L') && (pwm < 0)){
                analogWrite(motorL_for, 0);
                analogWrite(motorL_back, abs(pwm));
            }
        }
             
        void euler_to_quarternion(void){    // converts euler in Location struct to quarternion in Location struct
            currentLoc.qw = cos(currentLoc.roll) * cos(currentLoc.pitch) * cos(currentLoc.qy) - sin(currentLoc.roll) * sin(currentLoc.pitch) * sin(currentLoc.qy);
            currentLoc.qx = sin(currentLoc.roll) * sin(currentLoc.pitch) * cos(currentLoc.yaw) + cos(currentLoc.roll) * cos(currentLoc.pitch) * sin(currentLoc.yaw);
            currentLoc.qy = sin(currentLoc.roll) * cos(currentLoc.pitch) * cos(currentLoc.yaw) + cos(currentLoc.roll) * sin(currentLoc.pitch) * sin(currentLoc.yaw);
            currentLoc.qz = cos(currentLoc.roll) * sin(currentLoc.pitch) * cos(currentLoc.yaw) - sin(currentLoc.roll) * cos(currentLoc.pitch) * sin(currentLoc.yaw);
        }

        Coords pos_to_Grid(double x, double y){     //calculates which grid square the robot is in based on position
            struct Coords pos;
            pos.x = ceil(x / squareWidth);
            pos.y = ceil(y / squareHeight);

            return pos;
        }

        void grid_to_Pos(void){     //calculates which grid square the robot is in based on position
            currentLoc.Pos.x = currentLoc.Grid.x * squareWidth;
            currentLoc.Pos.y = currentLoc.Grid.y * squareHeight;
            desiredLoc.Pos.x = desiredLoc.Grid.x * squareWidth;
            desiredLoc.Pos.y = desiredLoc.Grid.y * squareHeight;
        }

        int readIR(int sensor){  //sends a signal with the IR sensor then detects how long it takes to come back
            pinMode(sensor, OUTPUT );
            digitalWrite(sensor, HIGH);
            delayMicroseconds(10);
            pinMode(sensor, INPUT);
            long reading = micros();

            //time how long the input is HIGH, but quit after 3ms as nothing happens after that
            while (digitalRead(sensor) == HIGH && micros() - reading < 3000);
            int diff = micros() - reading;
        
            return diff;
        }

};


#endif



// For passing structs into functions
// void data(Location *currentLoc) {
// void loop(){
//     int data(&samples);
// }