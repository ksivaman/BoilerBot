#ifndef __NAVIGATION_H__
#define __NAVIGATION_H__

#include "astar.h"
#include "AnalyzeLiDAR.h"



void adjustHeading(rover robot);
// int navigate(Path* pathStart, enum compass* currHeading, rover * robot1);
void navigate(rover * robot, Point dest);
int getBurstLen(Point start, Point dest);
void getOut(rover robot1, int currAngle, enum dir direction);
void fitInSqure(rover * robot);
void reposition(rover * robot);

int getTurnAngle(Path* path, enum compass* currHeading);


#endif