#include "astar.h"

void getRequest(char* buf, Point * start, Point * end);
int post_url_content(const char url[], char buf[], int length);
int get_url_content(const char url[], char buf[], int length);
void prints(char * p);