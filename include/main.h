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
        const int           width=94, height=9, diameter=65,    // mm (wheel center to center, IR to ground, wheel)
                            l=width/2, radius=diameter/2, 
                            gridWidth = 2000, gridHeight = gridWidth, 
                            squaresinGrid = 10,
                            squareWidth = gridWidth / squaresinGrid, squareHeight = squareWidth,
					        rpm_convert = 6000, ticks_per_rev = 20,         // odometry coefficients
                            viewThreshold = 3000, loopLength = 100;                            
        const String        name = BUILD_ENV_NAME;

        struct Location     currentLoc, desiredLoc;
        int                 id, velocity, rightOld=-1, leftOld=-1,
                            vel = 0, ang_vel = 0, viewDist, decision = 0; 
		double 	            theta = 0, phi = 0,               //radians
                            time_nowL, time_nowR, time_previousL, time_previousR, delta_t_R = 0, delta_t_L = 0,
                            omega_left = 0, omega_right = 0, v_center = 0, v_right = 0, v_left = 0, revL = 0, revR = 0,
                            k_rho = 4, k_alpha = 20, k_beta = -4,		// control coefficients      
                            IRThreshold, distThreshold = 20, rSign = 1, lSign = 1,
                            d_x, d_y, d_theta;	                      
        bool                active = false;    //pheromones
        uint8_t             currentPher[5], prevPher[5], scents[5]; 
        swarm_msgs::Grid    world;   // same as Grid being sent (no pheromones in obstacles so 'N' doesnt overwrite any data)
        Coords              path[30];

        ant(){
            currentLoc = {  0,0,
                            0,0,
                            0,0,0,
                            0,0,0,0};
            currentLoc.Pos.x = double(squareWidth);
            currentLoc.Pos.y = double(squareWidth);

            desiredLoc = {  0,0,
                            0,0,
                            0,0,0,
                            0,0,0,0};
            desiredLoc.Pos.x = double(squareWidth);
            desiredLoc.Pos.y = double(squareWidth);
        }

        void nextStep(String state){     
            bool avoidspace = false;       
            while(avoidspace == true || decision == 0){     // loop if the chosen space needs to be avoided ('N')
                if(state == "ROAM"){    // no food or pheromones found
                    // Serial.print("test1");
                    decision = randomSearch();
                }
                else if(state == "FOUND_FOOD"){     // located goal, see if anyone else has been there, if not: get help, if so: wait
                    active = true;
                    if(firstToFind()){
                        state = "GET_HELP";
                    }
                    else{
                        surroundFood();
                        state = "GROUP_UP";
                    }
                }
                else if(state == "GET_HELP"){   // go back to start & leave pheromones
                    desiredLoc.Pos.x = double(squareWidth);
                    desiredLoc.Pos.y = double(squareHeight);
                }
                else if(state == "FOLLOW_TRAIL"){   //found stronger pheromones, follow it back to food
                    
                }
                else if(state == ""){

                }
                else{   // if no state found
                    try{throw "NO STATE FOUND";} catch(int E){Serial.print("AN EXCEPTION WAS THROWN: "); Serial.print(E);}     // throw exception 
                }
                grid_to_Pos();
                avoidspace = avoidSpace(desiredLoc.Grid.x, desiredLoc.Grid.y);
                Serial.print("decision:"); Serial.print(decision); Serial.print("\t");
                Serial.print(avoidSpace(desiredLoc.Grid.x, desiredLoc.Grid.y)); Serial.print("\t");
            }
        }

        void surroundFood(void){
            while(avoidSpace(desiredLoc.Grid.x, desiredLoc.Grid.y)){    // loops until an open spot is found next to the food
                randomSearch();
            }
        }

        bool avoidSpace(int x, int y){    // make sure robot doesn't want to go out of bounds or into a wall
            return (world.column[x].row[y].pheromones[0] == 'N' || x >= squaresinGrid || y >= squaresinGrid|| x <= 0 || y <= 0) ? true : false;
        }

        int randomSearch(void){
            // wander around, avoid hitting walls
            
            int choice = random(1,100);
            uint8_t weights[] = {0, 10, 20, 31, 20, 10, 4, 1, 4}; //toal 100
            uint8_t weightsums[9] = {0};

            weightsums[0] = weights[0];
            for(uint8_t i=1; i < sizeof(weights); i++){
                weightsums[i] = weights[i] + weightsums[i-1];
            }
            if(weightsums[8] != 100){
                try{throw "WEIGHTS DON'T TOTAL 100";} catch(int E){Serial.print("AN EXCEPTION WAS THROWN: "); Serial.print(E);}     // throw exception 
            }
            // Serial.print("test3");

        /*  -------------
            | 4 | 3 | 2 |       ^
            -------------     North
            | 5 | C | 1 |   
            -------------
            | 6 | 7 | 8 |
            -------------   */

            if(choice >= 0 && choice <= weightsums[0]){                     // C
                desiredLoc.Grid.x = currentLoc.Grid.x + 0;
                desiredLoc.Grid.y = currentLoc.Grid.y + 0;
                return 0;
            }
            else if(choice >= weightsums[0] && choice <= weightsums[1]){    // 1
                desiredLoc.Grid.x = currentLoc.Grid.x + 1;
                desiredLoc.Grid.y = currentLoc.Grid.y + 0;
                return 1;
            }
            else if(choice >= weightsums[1] && choice <= weightsums[2]){    // 2
                desiredLoc.Grid.x = currentLoc.Grid.x + 1;
                desiredLoc.Grid.y = currentLoc.Grid.y + 1;
                return 2;
            }
            else if(choice >= weightsums[2] && choice <= weightsums[3]){    // 3
                desiredLoc.Grid.x = currentLoc.Grid.x + 0;
                desiredLoc.Grid.y = currentLoc.Grid.y + 1;
                return 3;
            }
            else if(choice >= weightsums[3] && choice <= weightsums[4]){    // 4
                desiredLoc.Grid.x = currentLoc.Grid.x - 1;
                desiredLoc.Grid.y = currentLoc.Grid.y + 1;
                return 4;
            }
            else if(choice >= weightsums[4] && choice <= weightsums[5]){    // 5
                desiredLoc.Grid.x = currentLoc.Grid.x - 1;
                desiredLoc.Grid.y = currentLoc.Grid.y + 0;
                return 5;
            }
            else if(choice >= weightsums[5] && choice <= weightsums[6]){    // 6
                desiredLoc.Grid.x = currentLoc.Grid.x - 1;
                desiredLoc.Grid.y = currentLoc.Grid.y - 1;
                return 6;
            }
            else if(choice >= weightsums[6] && choice <= weightsums[7]){    // 7
                desiredLoc.Grid.x = currentLoc.Grid.x - 1;
                desiredLoc.Grid.y = currentLoc.Grid.y - 0;
                return 7;
            }
            else if(choice >= weightsums[7] && choice <= weightsums[8]){    // 8
                desiredLoc.Grid.x = currentLoc.Grid.x + 1;
                desiredLoc.Grid.y = currentLoc.Grid.y - 1;
                return 8;
            }
            else{return 0;}
        }

        void getOdom(void){     // gets currentLoc info from encoders
            // right = velocity of right wheel, left = velocity of left wheel
            // double timeInc = loopLength/1000;
            if(millis() - time_nowL >= 500){v_left = 0;}
            if(millis() - time_nowR >= 500){v_right = 0;}
            
            double d_left = v_left * 0.1 * 100;        // convert to mm
            double d_right = v_right * 0.1 * 100;
            
            Serial.print("ReadVel: ");
            Serial.print(v_left); Serial.print("\t");
            Serial.print(v_right); Serial.print("\t");
            Serial.print("Dist Trav: ");
            Serial.print(d_left); Serial.print("\t");
            Serial.print(d_right); Serial.print("\t");
            // Serial.print(timeInc); Serial.print("\t");
            

            double d_center = (d_left + d_right)/2; //linear velocity of robot center
            phi = 2*PI*(d_left - d_right)/(width); //calculates change in yaw

            Serial.print("Result: ");
            Serial.print(d_center); Serial.print("\t");
            Serial.print(phi); Serial.print("\t");

            theta = theta + phi; // yaw = theta
            if(theta > 2*PI){
                currentLoc.yaw = theta - 2*PI;
                // theta = theta - 2*PI;
            }
            else if(theta < -2*PI){
                currentLoc.yaw = theta + 2*PI;
                // theta = theta + 2*PI;
            }
            else{
                currentLoc.yaw = theta;
                }

            
            Serial.print("Coord Result: ");
            Serial.print(d_center*cos(currentLoc.yaw)); Serial.print("\t");
            Serial.print(d_center*sin(currentLoc.yaw)); Serial.print("\t");

            currentLoc.Pos.x = currentLoc.Pos.x - d_center*cos(currentLoc.yaw); //new x pos from old x (updates once each loop (10Hz))
            currentLoc.Pos.y = currentLoc.Pos.y - d_center*sin(currentLoc.yaw); //new y pos from old y

            euler_to_quarternion(); // calculated quarternion fron existing euler
        }

        Coords controlLaw(void){
            Serial.print("test1"); Serial.print("\t");
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
            double beta = -d_theta - alpha;      
            // int beta = -alpha;      // make just -alpha so doesnt have to correct orientation?
            
            Serial.print("Coefs: ");
            Serial.print(rho); Serial.print("\t");
            Serial.print(alpha); Serial.print("\t");
            Serial.print(beta); Serial.print("\t");

            vel = k_rho*rho/1;   // mm/s -> m/s         //2
            ang_vel = k_alpha*alpha+k_beta*beta;        //15    -8

            // if ((abs(d_x)+abs(d_y)+abs(theta)) > threshold){
            //     // TODO
            // }

            struct Coords output;
            output.x = vel;
            output.y = ang_vel;
            return output;
        }

        void moveRobot(int vel, int ang_vel){
            int rVel, lVel, rdrive, ldrive;
            rVel = vel + (ang_vel * radius);
            lVel = vel - (ang_vel * radius);
            if(rVel < 0){rVel = 0;}
            if(lVel < 0){lVel = 0;}

            rVel = map(rVel, 0, 500, 50, 255); //convert to PWM that makes the motor turn
            lVel = map(lVel, 0, 500, 50, 255);

            // while(rVel > 0 && v_right == 0){             //make sure wheel is turning if it watns to be
            //     rVel++;
            //     driveWheel('R',rVel);
            // }   
            // while(lVel > 0 && v_left == 0){
            //     lVel++;
            //     driveWheel('L',lVel);           
            // }

            driveWheel('R',rVel);
            driveWheel('L',lVel);

            rSign = rVel/ abs(rVel);
            lSign = lVel/ abs(lVel);


            // while(rVel > 0 && v_right == 0){             //make sure wheel is turning if it watns to be
            //     rVel++;
            //     driveWheel('R',rVel);
            // }   
            // while(lVel > 0 && v_left == 0){
            //     lVel++;
            //     driveWheel('L',lVel);           
            // }

            Serial.print("WriteVels: ");
            Serial.print(rVel); Serial.print("\t");
            Serial.print(lVel); Serial.print("\t");

            // Serial.print("test2"); Serial.print("\t");
            // Serial.print(rVel); Serial.print("\t");
            // Serial.print(lVel); Serial.print("\t");

        }

        void driveWheel(char wheel, int pwm){
            if((wheel == 'r' || wheel == 'R') && (pwm >= 0)){
                analogWrite(motorR_for, pwm);
                analogWrite(motorR_back, 0);
            }
            else if((wheel == 'r' || wheel == 'R') && (pwm < 0)){
                analogWrite(motorR_for, 0);
                analogWrite(motorR_back, pwm);
            }
            else if((wheel == 'l' || wheel == 'L') && (pwm >= 0)){
                analogWrite(motorL_for, pwm);
                analogWrite(motorL_back, 0);
            }
            else if((wheel == 'l' || wheel == 'L') && (pwm < 0)){
                analogWrite(motorL_for, 0);
                analogWrite(motorL_back, pwm);
            }

            // if((wheel == 'r' || wheel == 'R') && (dir == 'f' || dir == 'F')){
            //     analogWrite(motorR_for, pwm);
            //     analogWrite(motorR_back, 0);
            // }
            // else if((wheel == 'r' || wheel == 'R') && (dir == 'b' || dir == 'B')){
            //     analogWrite(motorR_for, 0);
            //     analogWrite(motorR_back, pwm);
            // }
            // else if((wheel == 'l' || wheel == 'L') && (dir == 'f' || dir == 'F')){
            //     analogWrite(motorL_for, pwm);
            //     analogWrite(motorL_back, 0);
            // }
            // else if((wheel == 'l' || wheel == 'L') && (dir == 'b' || dir == 'B')){
            //     analogWrite(motorL_for, 0);
            //     analogWrite(motorL_back, pwm);
            // }
        }
             
        bool firstToFind(void){
            bool first;
            // if another bot has pheromones already there
            if(name == "Blue" && (currentPher[1] || currentPher[2] || currentPher[3] || currentPher[4])){ 
                first = false;
            }
            else if(name == "Charlie" && (currentPher[0] || currentPher[2] || currentPher[3] || currentPher[4])){
                first = false;
            }
            else if(name == "Delta" && (currentPher[0] || currentPher[1] || currentPher[3] || currentPher[4])){
                first = false;
            }
            else if(name == "Echo" && (currentPher[0] || currentPher[1] || currentPher[2] || currentPher[4])){
                first = false;
            }
            else if(name == "Foxtrot" && (currentPher[0] || currentPher[1] || currentPher[2] || currentPher[3])){
                first = false;
            }
            else{
                first = true;
            }
            return first;
        }

        void findHelp(void){
            active = true;
            // drive back to start, setting 'active' pheromones along the way

        }

        bool ReadIR(int sensor){  //sends a signal with the IR sensor then detects how long it takes to come back
            pinMode(sensor, OUTPUT );
            digitalWrite(sensor, HIGH);
            delayMicroseconds(10);
            pinMode(sensor, INPUT);
            long reading = micros();

            //time how long the input is HIGH, but quit after 3ms as nothing happens after that
            while (digitalRead(sensor) == HIGH && micros() - reading < 3000);
            int diff = micros() - reading;
        
            // Serial.print(diff); Serial.println("\t");
            return diff;
            // int threshold = 200;
            // if(diff >= threshold){      // TODO may want to seperate this into its own function...
            //     return(true);
            // }
            // else{ // if(diff < threshold)
            //     return(false);
            // }

        }

        bool avoidObstacle(int endGoal){
            // currentLoc.Pos.x;   // x
            // currentLoc.Pos.y;   // y
            // currentLoc.yaw;     // theta
            int distance = squareWidth;

            struct Coords inFront;
            inFront.x = distance * cos(currentLoc.yaw);
            inFront.y = distance * sin(currentLoc.yaw);

            inFront = pos_to_Grid(inFront.x, inFront.y);
            // 10 cm in both cases
            if(world.column[(int)inFront.x].row[(int)inFront.y].pheromones[0] == 'N'){  // if there is supposed to be an obstacle based on previous experience
                return true;
            }
            else if (viewDist <= 100){        // if there is an object that can be seen
                return true;
            }
            else{
                return false;
            }


            // if(distance <= viewThreshold){
            //     k_rho = 0, k_beta = 0;      // correct alpha first to see which way is faster around the obstacle (may only be effective if coming at an angle)
            // }
        }

        void locateObstacle(int distance){  //logs obstacle in personal world array
            if(distance <= viewThreshold){                   // if there's an obstacle to avoid (TODO find real threshold)
                // int squareCenterX = currentLoc.Grid.x * squareWidth - squareWidth/2;        //get center of grid square currently in
                // int squareCenterY = currentLoc.Grid.y * squareHeight - squareHeight/2;
                int obstacleX = distance * cos(currentLoc.yaw);
                int obstacleY = distance * sin(currentLoc.yaw);
                struct Coords pos = pos_to_Grid(obstacleX, obstacleY);
                world.column[int(pos.x)].row[int(pos.x)].pheromones[0] = 'N';   // indicates obstacle
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

        bool scentFilter(int scent){        // makes sure the high reading was not a blip and it actually has found food
            scents[4] = scents[3];
            scents[3] = scents[2];
            scents[2] = scents[1];
            scents[1] = scents[0];
            scents[0] = scent;
            //returns true if average is greater than threshold
            return ((scents[4] + scents[3] + scents[2] + scents[1] + scents[0])/5 > IRThreshold); 
        }

        void calibrateIR(int reading){      // calibrate margin to half way between initial reading(floor) and 3000(food)
            if(reading < 3000){
                IRThreshold = (3000 - reading) / 2;
            }
        }

        bool reachedGoal(void){
            // Serial.print("Dist ");
            // Serial.print(currentLoc.Pos.x - desiredLoc.Pos.x <= distThreshold); Serial.print("\t");
            // Serial.print(currentLoc.Pos.y - desiredLoc.Pos.y <= distThreshold); Serial.print("\t");
            // Serial.print(currentLoc.yaw - desiredLoc.yaw <= distThreshold); Serial.print("\t");
            return((abs(currentLoc.Pos.x - desiredLoc.Pos.x) <= distThreshold) && (abs(currentLoc.Pos.y - desiredLoc.Pos.y) <= distThreshold) && (abs(currentLoc.yaw - desiredLoc.yaw) <= distThreshold*0.03));
        }
};


#endif



// For passing structs into functions
// void data(Location *currentLoc) {
// void loop(){
//     int data(&samples);
// }