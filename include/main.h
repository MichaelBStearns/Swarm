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
                            distanceThreshold = 3000;                            
        const String        name = BUILD_ENV_NAME;

        struct Location     currentLoc, desiredLoc;
        int                 id, velocity, rightOld=-1, leftOld=-1,
                            vel = 0, ang_vel = 0, d_x, d_y, viewDist; 
		double 	            theta = 0, phi = 0,               //radians
                            time_nowL, time_nowR, time_previousL, time_previousR, delta_t_R = 0, delta_t_L = 0,
                            omega_left = 0, omega_right = 0, v_center = 0, v_right = 0, v_left = 0, revL = 0, revR = 0,
                            k_rho = 2, k_alpha = 15, k_beta = -8,		// control coefficients      
                            IRThreshold, distThreshold = 1000;	                      
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
            int decision = 0;
            
            // Serial.print("test2");
            while(avoidSpace(desiredLoc.Grid.x, desiredLoc.Grid.y) || decision == 0){     // loop if the chosen space needs to be avoided ('N')
                if(state == "ROAM"){    // no food or pheromones found
                    // Serial.print("test1");
                    decision = randomSearch();
                    Serial.print("decision:"); Serial.print(decision); Serial.print("\t");
                    Serial.print(avoidSpace(desiredLoc.Grid.x, desiredLoc.Grid.y)); Serial.print("\t");
                }
                else if(state == "FOUND_FOOD"){     // located goal, see if anyone else has been there, if not: get help, if so: wait
                    active = true;
                    if(firstToFind()){
                        state = "GET_HELP";
                        findHelp();
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
                    decision = strongestPher();
                    Serial.print("decision:"); Serial.print(decision); Serial.print("\t");
                    //move to new location
                }
                else if(state == ""){

                }
                else{   // if no state found
                    try{throw "NO DECISION STATE FOUND";} catch(int E){Serial.print("AN EXCEPTION WAS THROWN: "); Serial.print(E);}     // throw exception 
                }
            }
        }

        void surroundFood(void){
            while(avoidSpace(desiredLoc.Grid.x, desiredLoc.Grid.y)){    // loops until an open spot is found next to the food
                randomSearch();
            }
        }

        bool avoidSpace(int x, int y){    // make sure robot doesn't want to go out of bounds or into a wall
            return (world.column[x].row[y].pheromones[0] == 'N' || x >= squaresinGrid || y >= squaresinGrid) ? true : false;
        }

        void getOdom(void){     // gets currentLoc info from encoders
            // right = velocity of right wheel, left = velocity of left wheel

            v_center = (v_left + v_right)/2; //linear velocity of robot center
            phi = (v_right - v_right)/width; //calculates change in yaw
            currentLoc.yaw = currentLoc.yaw + phi; // yaw = theta
            if (currentLoc.yaw >= 2*PI){  //adjust currentLoc.yaw so that it is between 0 and 2pi
                while(currentLoc.yaw > 2*PI){
                    currentLoc.yaw = currentLoc.yaw - 2*PI;
                }
            } else if (currentLoc.yaw < 0){
                while(currentLoc.yaw < 0){
                    currentLoc.yaw = currentLoc.yaw + 2*PI;
                }
            }
            currentLoc.Pos.x = currentLoc.Pos.x + v_center*cos(theta); //new x pos from old x
            currentLoc.Pos.y = currentLoc.Pos.y + v_center*sin(theta); //new y pos from old y

            euler_to_quarternion(); // calculated quarternion fron existing euler
        }

        Coords controlLaw(void){
            Serial.print("test1"); Serial.print("\t");
            if(k_alpha + (5/3)*k_beta - (2/PI)*k_rho <= 0){     // if coefficients are not robustly stable
                try{throw "NO ROBUST STABILITY";} catch(int E){Serial.print("AN EXCEPTION WAS THROWN: "); Serial.print(E);}     // throw exception 
            }
            int threshold = 0.01;
            
            d_x = desiredLoc.Pos.x - currentLoc.Pos.x, 
            d_y = desiredLoc.Pos.y - currentLoc.Pos.y,
            theta = currentLoc.yaw; 

            int rho = sqrt((d_x)^2+(d_y)^2);
            int alpha = -theta + atan2(d_y, d_x);
            int beta = -theta - alpha;      
            // int beta = -alpha;      // make just -alpha so doesnt have to correct orientation?
            
            vel = k_rho*rho;
            ang_vel = k_alpha*alpha+k_beta*beta;

            if ((abs(d_x)+abs(d_y)+abs(theta)) > threshold){
                // TODO
            }

            struct Coords output;
            output.x = vel;
            output.y = ang_vel;
            return output;
        }

        void moveRobot(int vel, int ang_vel){
            int rVel, lVel;
            rVel = vel + (ang_vel * radius);
            lVel = vel - (ang_vel * radius);
            rVel = map(rVel, 0, 10, 0, 255);
            lVel = map(lVel, 0, 10, 0, 255);


            Serial.print("test2"); Serial.print("\t");
            Serial.print(rVel); Serial.print("\t");
            Serial.print(lVel); Serial.print("\t");


            if(rVel > 0){
                driveWheel('R','F',rVel);
            }
            else{
                driveWheel('R','B',rVel);
            }

            if(lVel > 0){
                driveWheel('L','F',lVel);
            }
            else{
                driveWheel('L','B',lVel);
            }
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
             
        int randomSearch(void){
            // wander around, avoid hitting walls
            
            int choice = random(1,100);
            uint8_t weights[] = {0, 10, 20, 31, 20, 10, 4, 4, 1}; //total 100
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
            // TODO dont choose it if == 'N'
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

        void findHelp(void){ // drive back to start, setting 'active' pheromones along the way
            active = true; //set pheromones active
            if(name == "Blue" && active == true){  //lay down pheromeones according to robot name
                currentPher[0] = 99;
            }
            else if(name == "Charlie" && active == true){
                currentPher[1] = 99;
            }
            else if(name == "Delta" && active == true){
                currentPher[2] = 99;
            }
            else if(name == "Echo" && active == true){
                currentPher[3] = 99;
            }
            else if(name == "Foxtrot" && active == true){
                currentPher[4] = 99;
            }
            //drive back to start
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


            // if(distance <= distanceThreshold){
            //     k_rho = 0, k_beta = 0;      // correct alpha first to see which way is faster around the obstacle (may only be effective if coming at an angle)
            // }
        }

        void locateObstacle(int distance){  //logs obstacle in personal world array
            if(distance <= distanceThreshold){                   // if there's an obstacle to avoid (TODO find real threshold)
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
            return(((currentLoc.Pos.x - desiredLoc.Pos.x) <= distThreshold) && ((currentLoc.Pos.y - desiredLoc.Pos.y) <= distThreshold) && ((currentLoc.Pos.x - desiredLoc.Pos.x) <= distThreshold));
        }

        int strongestPher(){
            int maxVal[] = {};
            int maxIndex = 0;
            for(int choice = 1; choice < 9; choice++){ //loop through possible next squares
                for (int i = 0; i < sizeof(currentPher); i++) { //loop through potential pheromones in squares
                    if (currentPher[i] > maxVal[choice]) { //if value is larger store it in max val based on square and save index
                        maxVal[choice] = currentPher[i];
                        maxIndex = i;
                    }
                }
            } 

            int largest = 0;
            for (int square = 0; square < sizeof(maxVal); square++) { //loop max values to find largest pheromone in all choices
                if (maxVal[square+1] > maxVal[square]) { 
                    largest = square;
                }
            }
            //return where new square is
            if(largest == 1){    // 1
                desiredLoc.Grid.x = currentLoc.Grid.x + 1;
                desiredLoc.Grid.y = currentLoc.Grid.y + 0;
                return 1;
            }
            else if(largest == 2){    // 2
                desiredLoc.Grid.x = currentLoc.Grid.x + 1;
                desiredLoc.Grid.y = currentLoc.Grid.y + 1;
                return 2;
            }
            else if(largest == 3){    // 3
                desiredLoc.Grid.x = currentLoc.Grid.x + 0;
                desiredLoc.Grid.y = currentLoc.Grid.y + 1;
                return 3;
            }
            else if(largest == 4){    // 4
                desiredLoc.Grid.x = currentLoc.Grid.x - 1;
                desiredLoc.Grid.y = currentLoc.Grid.y + 1;
                return 4;
            }
            else if(largest == 5){    // 5
                desiredLoc.Grid.x = currentLoc.Grid.x - 1;
                desiredLoc.Grid.y = currentLoc.Grid.y + 0;
                return 5;
            }
            else if(largest == 6){    // 6
                desiredLoc.Grid.x = currentLoc.Grid.x - 1;
                desiredLoc.Grid.y = currentLoc.Grid.y - 1;
                return 6;
            }
            else if(largest == 7){    // 7
                desiredLoc.Grid.x = currentLoc.Grid.x - 1;
                desiredLoc.Grid.y = currentLoc.Grid.y - 0;
                return 7;
            }
            else if(largest == 8){    // 8
                desiredLoc.Grid.x = currentLoc.Grid.x + 1;
                desiredLoc.Grid.y = currentLoc.Grid.y - 1;
                return 8;
            }
            else{return 0;}
        }
};


#endif



// For passing structs into functions
// void data(Location *currentLoc) {
// void loop(){
//     int data(&samples);
// }