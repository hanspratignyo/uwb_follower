#include "ros/ros.h"
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <sstream>
#include <fstream>
#include <string>
using namespace std;

fstream f;
string portadd;

int main(int argc, char **argv)
{
	int node;
	int count[6] = {};
	int distance[6];
	string str;
	stringstream stream1;
	cout << "started" << endl;
	f.open("/dev/ttyACM0" , fstream::in);
	if(!f.is_open()) {
		cout << "error opening port";
	}
	ros::init(argc, argv, "talker");

	ros::NodeHandle n;

	ros::Publisher chatter_pub1 = n.advertise<std_msgs::Int16>("distance1", 10);
	ros::Publisher chatter_pub2 = n.advertise<std_msgs::Int16>("distance2", 10);

	ros::Rate loop_rate(50);

	while (ros::ok()) {
		if (f.is_open()) {
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
				if (node ==4) {
					if (count[node] != distance[node]){
					std_msgs::Int16 msg;
					msg.data=distance[node];
					ROS_INFO("%d", msg.data);
				    	chatter_pub1.publish(msg);
				}
				count[node] = distance[node];
				}
				if (node ==5) {
					if (count[node] != distance[node]){
					std_msgs::Int16 msg;
					msg.data=distance[node];
					ROS_INFO("%d", msg.data);
				    	chatter_pub2.publish(msg);
				}
				count[node] = distance[node];
				}
				
			}
		} 
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
