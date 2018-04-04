
#include "ros/ros.h"
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
using namespace std;
int main(int argc, char **argv)
{
	fstream f;
	int node;
	int distance[6];
	string str;
	stringstream stream1;
	f.open("/dev/ttyACM0", fstream::in);
	if(!f.is_open()){ cout << "open error";}
	while(f >> str) {
				if (str.find(":")==1) {
					stream1.clear();
					stream1.str("");
					stream1.str(str);
					stream1 >> node;
				}
				if (str.find("mm")==2 ||str.find("mm")==3 || str.find("mm")==4 || str.find("mm")==5) {
					stream1.clear();
					stream1.str("");
					stream1.str(str);
					stream1 >> distance[node];
					cout << "distance of anchor " << node << " is " << distance[node] << endl;
				}

			}
	return 0;
}
