
#include "ros/ros.h"
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
using namespace std;

fstream f;
string portadd;
bool want_to_open = false;
bool port_opened = false;

void on_off_cb(const std_msgs::Bool trigger);
int main(int argc, char **argv)
{
	int node;
	int distance[6];
	string str;
	stringstream stream1;
	ros::param::get("~nodeid",node);
	ros::param::get("~portaddress",portadd);
	ros::init(argc, argv, "talker");

	ros::NodeHandle n;

	ros::Publisher chatter_pub = n.advertise<std_msgs::Int16>("chatter", 10);
	ros::Subscriber on_off_sub = n.subscribe<std_msgs::Bool>("/uwb_on_off", 10, on_off_cb);	

	ros::Rate loop_rate(50);

	int count = 0;
	while (ros::ok()) {
		if (port_opened) {
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
					//cout << "distance of anchor " << node << " is " << distance[node] << endl;
				}
				if (count != distance[node]){
					std_msgs::Int16 msg;
					msg.data=distance[node];
					ROS_INFO("%d", msg.data);
				    	chatter_pub.publish(msg);
				}
				count = distance[node];
			}
		} 
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

void on_off_cb(const std_msgs::Bool trigger){
	want_to_open = trigger.data;
	if (want_to_open){
		if (port_opened){
			ROS_INFO("PORT already opened");
		} else {			
			f.open(portadd.c_str(), fstream::in);
		}
	} else {
		if (port_opened){
			f.close();		
		} else {
			ROS_INFO("PORT already closed");
		}
	}
}
