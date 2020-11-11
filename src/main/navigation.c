#include "astar.h"

void navigate(Path* pathStart, enum compass* currHeading) {
    // Pointer to traverse through A* path
    Path* path = pathStart;
    // Navigate until at end of A* path (when path->next == NULL)
    while(path->next != NULL) {
        int turnAngle = getTurnAngle(path, currHeading);
        Path* burstStart = path;

        // TODO: Call Jin's turn function based on turnAngle

        // Find burst direction
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

        // Jin's approach
        int burstDir;
        int burstLen;
        // If moving in X Direction
        if(changeXDir != 0) {
            burstLen = abs(burstEndPoint.x - burstStart->data.x);
            // burstDir = changeXDir > 0? NORTH : SOUTH;
        }
        // If moving in Y Direction
        else {
            burstLen = abs(burstEndPoint.y - path_start->data.y);
            // burstDir = changeYDir > 0? WEST : EAST;
        }

        // TODO: burst(burstLen, burstEndPoint) // Pass in distance, and the final point. burst till you reach that point

        // Get current scan and quantize it
        // startScan(false, RPLIDAR_DEFAULT_TIMEOUT*2);
        // while(!(IS_OK(grabData(RPLIDAR_DEFAULT_TIMEOUT, buff))));
        // stop();
        // float* currScan = quantizeScan(node, NUM_SAMPLES);

        // Turn on motors
        // TODO: Turn on motors here

        // Go forward until burst end point reached
        // ******* Possible modification: subtract a distance from the burst endpoint distance measurement, in order to account for slight amount of rolling after motors stop
        // while(currScan[0] > lidar_data[burstEndPoint.x][burstEndPoint.y][0]) {
        //     // Get new quantized scan while bot is moving
        //     startScan(false, RPLIDAR_DEFAULT_TIMEOUT*2);
        //     while(!(IS_OK(grabData(RPLIDAR_DEFAULT_TIMEOUT, buff))));
        //     stop();
        //     currScan = quantizeScan(node, NUM_SAMPLES);
        // }

        // Burst end point was reached, turn off motors
        // TODO: Turn off motors here
    }
}


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

    int currDir;
    switch(*currHeading) {
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




// OLD TURNING LOGIC

// turnLeft == true: turn left          turnLeft == false: turn right
bool turnLeft;

// If burst was in Y direction
if(changeYDir != 0) {
    int newchangeDir = path->next->data.x - path->data.x;
    // If burst was in +Y direction
    if(changeYDir > 0) {
        turnLeft = newchangeDir > 0? false : true;
    }
    // If burst was in -Y direction
    else {
        turnLeft = newchangeDir > 0? true : false;
    }
}
// If burst was in X direction
else {
    int newchangeDir = path->next->data.y - path->data.y;
    // If burst was in +X direction
    if(changeXDir > 0) {
        turnLeft = newchangeDir > 0? true : false;
    }
    // If burst was in -X direction
    else {
        turnLeft = newchangeDir > 0? false : true;
    }
}