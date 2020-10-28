#include <stdio.h> 
#include <stdlib.h> 
#include <stdbool.h>

typedef struct point { //data structure representing 1 coordinate
    int x;
    int y;
} Point;
 
typedef struct node { //represents 1 node in the priority queue. Represented using Linked Lists
	Point data; 
	int priority; 
	struct node* next; 
} PQueue; 

typedef struct path {
	Point data;
	struct path* next;
} Path;

//Priority Queue related functions
PQueue* newNode(Point d, int p);
Point peek(PQueue** head);
Point pop(PQueue** head);
void push(PQueue** head, Point d, int p);
int isEmpty(PQueue** head);
void freePQueue(PQueue * pq);

//A* related functions
Path* getPathAStar(int numRows, int numCols, int grid[][numCols], Point start, Point end);
int getPriority(Point current, Point end);
void getNeighbors(Point node, Point *neighbors);
bool isPointValid(Point p, int numCols, int numRows);

//Path related functions
void append(Path ** head, Point d);
Path* newPath(Point d);
void printPath(Path * head);
void freePaths(Path *** paths, int numRows, int numCols);
void freePath(Path *path);
void reversePath(Path ** to, Path * from);

// #define NROWS 12
// #define NCOLS 12

// int fplan[NROWS][NCOLS] = {
// 	{0,0,0,0,0,0,0,0,0,0,0,0},
// 	{0,0,0,0,0,0,0,0,0,0,0,0},
// 	{1,1,1,1,1,1,1,1,1,1,1,0},
// 	{0,0,0,0,0,0,0,0,0,0,0,0},
// 	{0,0,0,0,0,0,0,0,0,0,0,0},
// 	{0,0,0,0,0,0,0,0,0,0,0,0},
// 	{0,1,1,1,1,1,1,1,1,1,1,1},
// 	{0,1,1,1,1,1,1,1,1,1,1,1},
// 	{0,0,0,0,0,0,0,0,0,0,0,0},
// 	{0,0,0,0,0,0,0,0,0,0,0,0},
// 	{0,0,0,0,0,0,0,0,0,0,0,0},
// 	{0,0,0,0,0,0,0,0,0,0,0,0}
// };

#define NROWS 7
#define NCOLS 7

//floorplan data structure
int fplan[NROWS][NCOLS] = {
	{1,1,1,1,0,0,1},
	{0,0,0,1,0,0,1},
	{1,0,0,1,0,0,1},
	{1,0,0,0,0,0,0},
	{1,1,1,0,1,0,1},
	{1,1,1,0,1,0,1},
	{1,1,1,0,0,0,1}
};

