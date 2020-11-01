#include "include/astar.h"


Path* getPathAStar(int numRows, int numCols, int grid[][numCols], Point start, Point end) {

	Path * finalPath = NULL;

	bool ** seen = (bool**)malloc(numRows*sizeof(bool*));
  	for(int i=0; i<numRows; i++)
    	seen[i] = (bool*)malloc(numCols*sizeof(bool));

	Path *** paths = (Path***)malloc(numRows*sizeof(Path**));
  	for(int i=0; i<numRows; i++)
    	paths[i] = (Path**)malloc(numCols*sizeof(Path*));


	for (int i=0; i<numRows; i++) {
		for (int j = 0; j<numCols; j++) {
			seen[i][j] = false;
			paths[i][j] = NULL;
		}
	}

	PQueue * queue = newNode(start, getPriority(start, end));
	seen[start.x][start.y] = true;
	paths[start.x][start.y] = newPath(start);

	while (!isEmpty(&queue)) {
		Point node = pop(&queue);
		if (node.x == end.x && node.y == end.y) {
			reversePath(&finalPath, paths[node.x][node.y]);
			for (int i = 0; i < numRows; i++)
				free(seen[i]);
			free(seen); 
			freePaths(paths, numRows, numCols);
			freePQueue(queue);
			return finalPath;
		}
		Point neighbors[4];
		getNeighbors(node, neighbors);
		for (int i = 0; i < 4; i++) {
			if (!isPointValid(neighbors[i], numCols, numRows)) continue;
			if (seen[neighbors[i].x][neighbors[i].y]) continue;
			if (grid[neighbors[i].x][neighbors[i].y] == 1) continue;
			seen[neighbors[i].x][neighbors[i].y] = true;
			if (!isEmpty(&queue)) {
				paths[neighbors[i].x][neighbors[i].y] = newPath(neighbors[i]);
				(paths[neighbors[i].x][neighbors[i].y])->next = paths[node.x][node.y];
				push(&queue, neighbors[i], getPriority(neighbors[i], end));
			} else {
				paths[neighbors[i].x][neighbors[i].y] = newPath(neighbors[i]);
				(paths[neighbors[i].x][neighbors[i].y])->next = paths[node.x][node.y];
				queue = newNode(neighbors[i], getPriority(neighbors[i], end));
			}
		}
	}

	for (int i = 0; i < numRows; i++)
		free(seen[i]);
    free(seen); 
	freePaths(paths, numRows, numCols);
	return NULL;
}

bool isPointValid(Point p, int numCols, int numRows) {
	if (p.x < 0 || p.x >= numRows) {
		return false;
	} else if (p.y < 0 || p.y >= numCols) {
		return false;
	}
	return true;
}

void getNeighbors(Point node, Point *neighbors) {
	neighbors[0].x = node.x;
	neighbors[0].y = node.y + 1;
	neighbors[1].x = node.x;
	neighbors[1].y = node.y - 1;
	neighbors[2].x = node.x + 1;
	neighbors[2].y = node.y;
	neighbors[3].x = node.x - 1;
	neighbors[3].y = node.y;
}

//gets the priority of a point
int getPriority(Point current, Point end) {
    int priority = abs(end.y - current.y) + abs(end.x - current.x);
    return priority;
}

// Function to Create A New PQueue 
PQueue* newNode(Point d, int p) 
{ 
	PQueue* temp = (PQueue*)malloc(sizeof(PQueue)); 
	temp->data = d; 
	temp->priority = p; 
	temp->next = NULL; 

	return temp; 
} 

// Return the value at head 
Point peek(PQueue** head) 
{ 
	return (*head)->data; 
} 

// Removes the element with the highest priority form the list 
Point pop(PQueue** head) 
{ 
	PQueue* temp = *head;
	(*head) = (*head)->next; 
	Point value;
	value.x = (temp->data).x;
	value.y = (temp->data).y;
	free(temp); 
	return value;
} 

// Function to push according to priority 
void push(PQueue** head, Point d, int p) 
{ 
	PQueue* start = (*head); 

	// Create new Node 
	PQueue* temp = newNode(d, p); 

	// Special Case: The head of list has lesser priority than new node. So insert new node before head node and change head node. 
	if ((*head)->priority > p) { 
		// Insert New Node before head 
		temp->next = *head; 
		(*head) = temp; 
	} 
	else { 
		// Traverse the list and find a position to insert new node 
		while (start->next != NULL && 
			start->next->priority < p) { 
			start = start->next; 
		} 

		// Either at the ends of the list 
		// or at required position 
		temp->next = start->next; 
		start->next = temp; 
	} 
} 

// Function to check is list is empty 
int isEmpty(PQueue** head) 
{ 
	return (*head) == NULL; 
} 

// Function to Create A New Path node (1 point) 
Path* newPath(Point d) { 
	Path* temp = (Path*)malloc(sizeof(Path)); 
	temp->data = d; 
	temp->next = NULL; 
	return temp; 
} 

void append(Path ** head, Point d) {
	if ((*head) == NULL) {
		*head = newPath(d);
	} else {
		Path * temp = newPath(d);
		temp->next = *head;
		*head = temp;
	}
}

void printPath(Path * head) {
	Path * temp = head;
	printf("start -> ");
	while (temp) {
		printf("(%d, %d) -> ", (temp->data).x, (temp->data).y);
		temp = temp->next;
	}
	printf("end\n");
}

void freePath(Path *path) {
	Path* temp = path;
	while (temp) {
		path = temp->next;
		free(temp);
		temp = path;
	}
}

void freePQueue(PQueue * pq) {
	PQueue* temp = pq;
	while (temp) {
		pq = temp->next;
		free(temp);
		temp = pq;
	}
}

void freePaths(Path *** paths, int numRows, int numCols) {
	// for (int i=0; i<numRows; i++) {
	// 	for (int j = 0; j<numCols; j++) {
	// 		freePath(paths[i][j]);
	// 	}
	// }
	for(int i=0; i<numRows; i++)
    	free(paths[i]);
	free(paths);
}

void reversePath(Path ** to, Path * from) {
	*to = newPath(from->data);
	Path* tempFrom = from->next;
	while (tempFrom) {
		Path* temp = newPath(tempFrom->data);
		temp->next = *to;
		*to = temp;
		tempFrom = tempFrom->next;
	}
}

