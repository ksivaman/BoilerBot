#include "astar.h"
#include "AnalyzeLiDAR.h"

void navigate(Path* pathStart, enum compass* currHeading);
int getTurnAngle(Path* path, enum compass* currHeading);
