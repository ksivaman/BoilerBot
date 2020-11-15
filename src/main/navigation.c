#include "include/astar.h"
#include "include/AnalyzeLiDAR.h"
#include "include/navigation.h"

void reposition(rover * robot) {
    Point secondClose = {6,6};
    int angle; 
    Point curr;
    Point curr_input; 
    
    curr = get_curr_loc(&angle, &secondClose);
    printf("curr: (%d, %d)\n", curr.x, curr.y);
    if (!isPointEqual(robot->currLoc, secondClose)) { // when it is not boot up initialization
        if (!isPointEqual(curr, robot->currLoc)) {
            vTaskDelay(1000/ portTICK_PERIOD_MS);
            curr = get_curr_loc(&angle, &secondClose);
        }
    }

    printf("angle is %d\n", angle);

    int turn = robot->heading - angle;
    if (abs(turn) > 180) {
        if (turn > 0) {
            turn = 360 - turn;
        } else {
            turn = turn + 360;
        }
    }
    printf("__________ Correcting turn by %d degrees\n", turn);
    if (abs(turn) > 2 ) turn_rover(*robot, turn, RIGHT);
    adjustHeading(*robot);

    robot->currLoc = curr;

    fitInSqure(robot);
    turn_rover(*robot, 90, RIGHT);
    robot->heading = ((robot->heading) + 90) % 360;
    fitInSqure(robot);
}


int getBurstLen(Point start, Point dest) {
    if(start.x - dest.x != 0) {
        return abs(dest.x - start.x ) * SQUARE_WIDTH;
        // burstDir = changeXDir > 0? NORTH : SOUTH;
    }
    // If moving in Y Direction
    else {
        return abs(dest.y - start.y) * SQUARE_WIDTH;
        // burstDir = changeYDir > 0? WEST : EAST;
    }
}

void getOut(rover robot1, int currAngle, enum dir direction) {
    int turned = abs(currAngle);
    int delta = 5;
    while (turned < 30) {
        turn_rover(robot1, delta, direction);
        backOff(robot1);
        turned += delta;
        delta += 5;
    }
    turn_rover(robot1, (-1) * turned, direction);
}


void fitInSqure(rover * robot1) {
    int offset_n = 0;
    int offset_e = 0;
    int move = 0;

    getOffsets(*robot1, &offset_n, &offset_e);
    printf("North %d, East: %d\n", offset_n, offset_e);
    if ((robot1->heading == EAST || robot1->heading == WEST)) {
        while (abs(offset_e) > 2) {
            if (abs(offset_e) > OBSTACLE_FREE_BOUND) {
                do {
                    move = OBSTACLE_FREE_BOUND;
                    printf("move: %d\n", move);
                    burst_rover(*robot1, move, (offset_e > 0) ? WEST : EAST);
                    offset_e = (offset_e > 0) ? (offset_e - OBSTACLE_FREE_BOUND) : (offset_e + OBSTACLE_FREE_BOUND);
                } while (abs(offset_e) > OBSTACLE_FREE_BOUND);
            }
            burst_rover(*robot1, abs(offset_e), (offset_e > 0) ? WEST : EAST);
            getOffsets(*robot1, &offset_n, &offset_e);
        }
    } else {
        while (abs(offset_n) > 2) {
            if (abs(offset_n) > OBSTACLE_FREE_BOUND) {
                do {
                    move = OBSTACLE_FREE_BOUND;
                    printf("move: %d\n", move);
                    burst_rover(*robot1, move, (offset_n > 0) ? SOUTH : NORTH);
                    offset_n = (offset_n > 0) ? (offset_n - OBSTACLE_FREE_BOUND) : (offset_n + OBSTACLE_FREE_BOUND);
                } while (abs(offset_n) > OBSTACLE_FREE_BOUND);
            }
            burst_rover(*robot1, abs(offset_n), (offset_n > 0) ? SOUTH : NORTH);
            getOffsets(*robot1, &offset_n, &offset_e);
        }
    }
}


// expect robot to be heading the correct direction.
void moveToPoint(rover * robot1, Point dest) {
    int burstLen;
    float moved;
    int angle;
    int offset = 0;
    int offset_n = 0;
    int offset_e = 0;
    Point secondClose;

    do {
        burstLen = getBurstLen(robot1->currLoc, dest) + offset;
        printf("__________ Bursting %d cm to (%d, %d)\n", (int) (burstLen / SQUARE_WIDTH), dest.x, dest.y);
        moved = burst_rover(*robot1, burstLen, robot1->heading);
        robot1->currLoc = get_curr_loc_input(robot1->heading, &angle, &secondClose);

        if (isPointEqual(robot1->currLoc, dest)) // if we are in dest, break out
            break;

        // not at destination yet
        if (moved == -1) { // Stopped because of obstacle or fail in distance calculation.
            
            int offset_a = getAngleOffset(*robot1);
            float * lidarScan = getLiDARScan();
            float cm_right = fabs(offsetHelper(lidarScan, 60, 0, EAST, 1));
            float cm_left = fabs(offsetHelper(lidarScan, 60, 0, WEST, 1));
            bool obstacle_a = isThereObstacle_a(lidarScan, -offset_a);
            bool obstacle_r = isThereObstacle_r(lidarScan, 0, FORWARD);

            float extra = fabs(sin(offset_a / 180.0 * M_PI)) * 17.0;
            
            if (obstacle_r || obstacle_a) { // stopped due to obstacle
                if (!obstacle_a || ((cm_right + extra) < 21.0 ) || ((cm_left + extra) < 21.0)) { // bot is too close to wall 
                    enum dir turn = RIGHT;
                    if (offset_a != 0) {
                        turn = (offset_a > 0 ) ? RIGHT : LEFT;
                    } else {
                        turn = (cm_right < cm_left) ? RIGHT : LEFT;
                    }
                    getOut(*robot1, offset_a, turn);
                } else { // we see obstacle in front, wait till obstacle disappear
                    int64_t waitTime = esp_timer_get_time();
                    int64_t newTime = esp_timer_get_time(); 

                    while(obstacle_r && ((double)(newTime - waitTime) < (double) OBSTACLE_WAIT_DURATION)) {
                        updateLiDARScan(lidarScan);
                        obstacle_r = isThereObstacle_r(lidarScan, 0, FORWARD);
                        newTime = esp_timer_get_time();   
                    }
                    if ((float)(newTime - waitTime) >= (float) OBSTACLE_WAIT_DURATION) {
                        getOut(*robot1, offset_a, (cm_right < cm_left) ? RIGHT : LEFT);
                    }
                }
            }
            robot1->currLoc = get_curr_loc_input(robot1->heading, &angle, &secondClose);
            int turn = robot1->heading-angle;
            if (abs(turn) > 180) {
                if (turn > 0) {
                    turn = 360 - turn;
                } else {
                    turn = turn + 360;
                }
            }
            if (abs(turn) > 30 ) {
                turn_rover(*robot1, turn, RIGHT);
                adjustHeading(*robot1);
            } else if (abs(turn) > 3) {
                adjustHeading(*robot1);
            }
            
        }   

        if (robot1->heading == EAST || robot1->heading == WEST) {
            if (robot1->currLoc.x != dest.x) {
                break;
            }
            getOffsets(*robot1, &offset_n, &offset_e);
            if (abs(offset_e) > 2) {
                offset = (robot1->heading == EAST) ? (-1) * offset_e : offset_e;
            }
        } else {
            if (robot1->currLoc.y != dest.y) {
                break;
            }
            getOffsets(*robot1, &offset_n, &offset_e);
            if (abs(offset_n) > 2) {
                offset = (robot1->heading == NORTH) ? (-1) * offset_n : offset_n;
            }
        }

    } while (!isPointEqual(robot1->currLoc, dest));
    fitInSqure(robot1);
}


// need the currLoc of robot to be accurate
void adjustHeading(rover robot) {
    int angleOffset = getAngleOffset(robot);
    printf("__________ Correcting turn by %d degrees\n", angleOffset);
    if (abs(angleOffset) > 2 ){
        printf("__________ Correcting turn by %d degrees\n", angleOffset);
        turn_rover(robot, -angleOffset , RIGHT);
    }
}

// int navigate(Path* pathStart, enum compass* currHeading, rover * robot1) {
//     // Pointer to traverse through A* path
//     Path* path = pathStart;
//     // Navigate until at end of A* path (when path->next == NULL)
//     currHeading = &(robot1->heading);
//     // Navigate until at end of A* path (when path->next == NULL)
//     while(path->next != NULL) {
//         // Appropriately Turn
//         Path* burstStart = path;
//         robot1->currLoc.x = path->data.x;
//         robot1->currLoc.y = path->data.y;
//         int turnAngle = getTurnAngle(path, currHeading);
//         printf("__________ Turning %d degrees\n", turnAngle);
//         turn_rover(*robot1, turnAngle, RIGHT);

//         adjustHeading(robot1);
//         float * newScan = getLiDARScan();
//         int angle;
//         absoluteErrorFrom(newScan, path->data, &angle);
//         free(newScan);
//         printf("__________ Correcting turn by %d degrees\n", *currHeading-angle);
//         int subangle = 0;
//         if (angle > 180){
//             angle -= 360;
//         }
//         if (*currHeading == 270){
//             subangle = -90;
//         }
//         else {
//             subangle = *currHeading;
//         }
//         if (abs(subangle - angle) > 5 )
//             turn_rover(*robot1, subangle-angle , RIGHT);
          

//         // vTaskDelay(7000 / portTICK_PERIOD_MS);

//         // Find burst direction
//         int changeXDir = path->next->data.x - path->data.x;
//         int changeYDir;
//         if(changeXDir == 0) {           // If change isn't in X for current --> next, must be in Y
//             changeYDir = path->next->data.y - path->data.y;
//         }

//         // Set point as next point in path (to see if pattern continues beyond 1-square burst)
//         path = path->next;

//         // Go through A* path, starting from point after our current location, to find what point the burst ends at
//         // Checks path->next != NULL to make sure it's not at end of path
//         while(path->next != NULL) {
//             int newchangeDir = 0;
//             // If moving in X direction
//             if(changeXDir != 0) {
//                 newchangeDir = path->next->data.x - path->data.x;
//                 // If movement direction matches burst direction, update path
//                 if(newchangeDir == changeXDir) {
//                     path = path->next;
//                 }
//                 // If movement direction no longer matches burst direction (i.e. there's a turn)
//                 else {
//                     break;
//                 }
//             }
//             // If moving in Y direction
//             else {
//                 newchangeDir = path->next->data.y - path->data.y;
//                 // If movement direction matches burst direction, update path
//                 if(newchangeDir == changeYDir) {
//                     path = path->next;
//                 }
//                 // If movement direction no longer matches burst direction (i.e. there's a turn)
//                 else {
//                     break;
//                 }
//             }
//         }

//         Point burstEndPoint = path->data;

//         // Jin's approach
//         // int burstDir;
//         int burstLen;
//         // If moving in X Direction
//         if(changeXDir != 0) {
//             burstLen = abs(burstEndPoint.x - burstStart->data.x);
//             // burstDir = changeXDir > 0? NORTH : SOUTH;
//         }
//         // If moving in Y Direction
//         else {
//             burstLen = abs(burstEndPoint.y - burstStart->data.y);
//             // burstDir = changeYDir > 0? WEST : EAST;
//         }

//         printf("__________ Bursting %d blocks to (%d, %d)\n", burstLen, burstEndPoint.x, burstEndPoint.y);
//         bool obstacle = false;
//         robot1->heading = *currHeading;
//         if( burst_rover(*robot1, burstLen * SQUARE_WIDTH, *currHeading) == -1 || obstacle){
//             return -1;
//         }

//         // TODO: burst(burstLen, burstEndPoint) // Pass in distance, and the final point. burst till you reach that point

//         // Get current scan and quantize it
//         // startScan(false, RPLIDAR_DEFAULT_TIMEOUT*2);
//         // while(!(IS_OK(grabData(RPLIDAR_DEFAULT_TIMEOUT, buff))));
//         // stop();
//         // float* currScan = quantizeScan(node, NUM_SAMPLES);

//         // Turn on motors
//         // TODO: Turn on motors here

//         // Go forward until burst end point reached
//         // ******* Possible modification: subtract a distance from the burst endpoint distance measurement, in order to account for slight amount of rolling after motors stop
//         // while(currScan[0] > lidar_data[burstEndPoint.x][burstEndPoint.y][0]) {
//         //     // Get new quantized scan while bot is moving
//         //     startScan(false, RPLIDAR_DEFAULT_TIMEOUT*2);
//         //     while(!(IS_OK(grabData(RPLIDAR_DEFAULT_TIMEOUT, buff))));
//         //     stop();
//         //     currScan = quantizeScan(node, NUM_SAMPLES);
//         // }

//         // Burst end point was reached, turn off motors
//         // TODO: Turn off motors here
//     }
//     return 0;
// }

void navigate(rover * robot, Point dest) {
    Path* path = getPathAStar(NROWS, NCOLS, fplan, robot->currLoc, dest);
    printf("Start: (%d, %d); End: (%d, %d)\n", robot->currLoc.x, robot->currLoc.y, dest.x, dest.y);
    printPath(path);

    while(!isPointEqual(robot->currLoc, dest)) {
        int turnAngle = getTurnAngle(path, &(robot->heading));
        printf("__________ Turning %d degrees\n", turnAngle);
        turn_rover(*robot, turnAngle, RIGHT);
        adjustHeading(*robot);

        Path* burstStart = path;
        int changeXDir = path->next->data.x - path->data.x;
        int changeYDir;
        if(changeXDir == 0) {           // If change isn't in X for current --> next, must be in Y
            changeYDir = path->next->data.y - path->data.y;
        }
        // Set point as next point in path (to see if pattern continues beyond 1-square burst)
        path = path->next;

        // Go through A* path, starting from point after our current location, to find what point the burst ends at
        // Checks path->next != NULL to make sure it's not at end of path
        while(path->next != NULL) {
            int newchangeDir = 0;
            // If moving in X direction
            if(changeXDir != 0) {
                newchangeDir = path->next->data.x - path->data.x;
                // If movement direction matches burst direction, update path
                if(newchangeDir == changeXDir) {
                    path = path->next;
                }
                // If movement direction no longer matches burst direction (i.e. there's a turn)
                else {
                    break;
                }
            }
            // If moving in Y direction
            else {
                newchangeDir = path->next->data.y - path->data.y;
                // If movement direction matches burst direction, update path
                if(newchangeDir == changeYDir) {
                    path = path->next;
                }
                // If movement direction no longer matches burst direction (i.e. there's a turn)
                else {
                    break;
                }
            }
        }
        Point burstEndPoint = path->data;
        printf("calling MovingToPoint\n");
        moveToPoint(robot, burstEndPoint);

        path = getPathAStar(NROWS, NCOLS, fplan, robot->currLoc, dest);
    }
}


// void navigate(Path* pathStart, rover * robot1) {
//     // Pointer to traverse through A* path
//     Path* path = pathStart;
//     enum compass * currHeading = &(robot1->heading);
//     // Navigate until at end of A* path (when path->next == NULL)
//     while(path->next != NULL) {
//         // Appropriately Turn
//         int turnAngle = getTurnAngle(path, currHeading);
//         printf("__________ Turning %d degrees\n", turnAngle);
//         turn_rover(*robot1, turnAngle, RIGHT);

//         adjustHeading(robot1);
//         float * newScan = getLiDARScan();
//         int angle;
//         absoluteErrorFrom(newScan, path->data, &angle);
//         free(newScan);
//         printf("__________ Correcting turn by %d degrees\n", *currHeading-angle);
//         int subangle = 0;
//         if (angle > 180){
//             angle -= 360;
//         }
//         if (*currHeading == 270){
//             subangle = -90;
//         }
//         else {
//             subangle = *currHeading;
//         }
//         if (abs(subangle - angle) > 5 )
//             turn_rover(*robot1, subangle-angle , RIGHT);
          
//         // Find burst direction
//         Path* burstStart = path;
//         // robot1->currLoc.x = path->data.x;
//         // robot1->currLoc.y = path->data.y;

//         int changeXDir = path->next->data.x - path->data.x;
//         int changeYDir;
//         if(changeXDir == 0) {           // If change isn't in X for current --> next, must be in Y
//             changeYDir = path->next->data.y - path->data.y;
//         }
//         // Set point as next point in path (to see if pattern continues beyond 1-square burst)
//         path = path->next;

//         // Go through A* path, starting from point after our current location, to find what point the burst ends at
//         // Checks path->next != NULL to make sure it's not at end of path
//         while(path->next != NULL) {
//             int newchangeDir = 0;
//             // If moving in X direction
//             if(changeXDir != 0) {
//                 newchangeDir = path->next->data.x - path->data.x;
//                 // If movement direction matches burst direction, update path
//                 if(newchangeDir == changeXDir) {
//                     path = path->next;
//                 }
//                 // If movement direction no longer matches burst direction (i.e. there's a turn)
//                 else {
//                     break;
//                 }
//             }
//             // If moving in Y direction
//             else {
//                 newchangeDir = path->next->data.y - path->data.y;
//                 // If movement direction matches burst direction, update path
//                 if(newchangeDir == changeYDir) {
//                     path = path->next;
//                 }
//                 // If movement direction no longer matches burst direction (i.e. there's a turn)
//                 else {
//                     break;
//                 }
//             }
//         }
//         Point burstEndPoint = path->data;

//         printf("calling MovingToPoint\n");

//         moveToPoint(robot1, burstEndPoint);

//         // // Jin's approach
//         // // int burstDir;
//         // int burstLen;
//         // // If moving in X Direction
//         // if(changeXDir != 0) {
//         //     burstLen = abs(burstEndPoint.x - burstStart->data.x);
//         //     // burstDir = changeXDir > 0? NORTH : SOUTH;
//         // }
//         // // If moving in Y Direction
//         // else {
//         //     burstLen = abs(burstEndPoint.y - burstStart->data.y);
//         //     // burstDir = changeYDir > 0? WEST : EAST;
//         // }

//         // printf("__________ Bursting %d blocks to (%d, %d)\n", burstLen, burstEndPoint.x, burstEndPoint.y);
//         // bool obstacle = false;
//         // // robot1->heading = *currHeading;
//         // burst_rover(*robot1, burstLen * SQUARE_WIDTH, *currHeading, &obstacle);

//         // TODO: burst(burstLen, burstEndPoint) // Pass in distance, and the final point. burst till you reach that point

//         // Get current scan and quantize it
//         // startScan(false, RPLIDAR_DEFAULT_TIMEOUT*2);
//         // while(!(IS_OK(grabData(RPLIDAR_DEFAULT_TIMEOUT, buff))));
//         // stop();
//         // float* currScan = quantizeScan(node, NUM_SAMPLES);

//         // Turn on motors
//         // TODO: Turn on motors here

//         // Go forward until burst end point reached
//         // ******* Possible modification: subtract a distance from the burst endpoint distance measurement, in order to account for slight amount of rolling after motors stop
//         // while(currScan[0] > lidar_data[burstEndPoint.x][burstEndPoint.y][0]) {
//         //     // Get new quantized scan while bot is moving
//         //     startScan(false, RPLIDAR_DEFAULT_TIMEOUT*2);
//         //     while(!(IS_OK(grabData(RPLIDAR_DEFAULT_TIMEOUT, buff))));
//         //     stop();
//         //     currScan = quantizeScan(node, NUM_SAMPLES);
//         // }

//         // Burst end point was reached, turn off motors
//         // TODO: Turn off motors here
//     }
// }


int getTurnAngle(Path* path, enum compass* currHeading) {
    Point desiredDir = {.x = path->next->data.x - path->data.x, .y = path->next->data.y - path->data.y};
    int desiredAngle;
    if (desiredDir.x == 1 && desiredDir.y == 0){
        desiredAngle = 0;
    }
    else if (desiredDir.x == 0 && desiredDir.y == -1){
        desiredAngle = 90;
    }
    else if (desiredDir.x == -1 && desiredDir.y == 0){
        desiredAngle = 180;
    }
    else {
        desiredAngle = 270;
    }

    int currDir = -1;
    switch((*currHeading)) {
        case NORTH : currDir = 0;
                     break;
        case EAST:   currDir = 90;
                     break;
        case SOUTH:  currDir = 180;
                     break;
        case WEST:   currDir = 270;
                     break;
    }

    switch(desiredAngle) {
        case 0 :  *currHeading = NORTH;
                  break;
        case 90:  *currHeading = EAST;
                  break;
        case 180: *currHeading = SOUTH;
                  break;
        case 270: *currHeading = WEST;
                  break;
    }

    int turnAngle = desiredAngle - currDir;
    if(turnAngle == 270) {
        turnAngle = -90;
    }
    else if(turnAngle == -270) {
        turnAngle = 90;
    }

    return turnAngle;
}




// // OLD TURNING LOGIC

// // turnLeft == true: turn left          turnLeft == false: turn right
// bool turnLeft;

// // If burst was in Y direction
// if(changeYDir != 0) {
//     int newchangeDir = path->next->data.x - path->data.x;
//     // If burst was in +Y direction
//     if(changeYDir > 0) {
//         turnLeft = newchangeDir > 0? false : true;
//     }
//     // If burst was in -Y direction
//     else {
//         turnLeft = newchangeDir > 0? true : false;
//     }
// }
// // If burst was in X direction
// else {
//     int newchangeDir = path->next->data.y - path->data.y;
//     // If burst was in +X direction
//     if(changeXDir > 0) {
//         turnLeft = newchangeDir > 0? true : false;
//     }
//     // If burst was in -X direction
//     else {
//         turnLeft = newchangeDir > 0? false : true;
//     }
// }
