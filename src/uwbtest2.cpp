#include "ros/ros.h"
#include <beginner_tutorials/Int32Stamped.h>
#include <std_msgs/Bool.h>
#include <sstream>
#include <fstream>
#include <string>
#include <iostream>
using namespace std;

fstream f;
string port_addr = "/dev/ttyACM0";
bool want_to_open = false;

void on_off_cb(const std_msgs::Bool trigger);

int main(int argc, char **argv){
	ros::init(argc, argv, "talker");

	int testnode;
	int node_id[5] = {};
	int counts[6] = {};
	int distances[6];
	string str;
	string port_addr;
	stringstream stream1;
	ROS_INFO("[UWB_TEST]: Node started");
	
	ros::NodeHandle n;
	ros::NodeHandle nh_param("~");

	nh_param.param<int>("node_id4", node_id[2], node_id[2]);
	nh_param.param<int>("node_id5", node_id[3], node_id[3]);
	nh_param.param<int>("node_id3", node_id[1], node_id[1]);
	nh_param.param<int>("node_id6", node_id[4], node_id[4]);
	nh_param.param<int>("node_id2", node_id[0], node_id[0]);
	nh_param.param<string>("serial_port", port_addr, port_addr);

	ROS_INFO("[UWB_TEST]: Serial port: %s", port_addr.c_str());

	ros::Publisher chatter_pub1 = n.advertise<beginner_tutorials::Int32Stamped>("distance1", 10);
	ros::Publisher chatter_pub2 = n.advertise<beginner_tutorials::Int32Stamped>("distance2", 10);
	ros::Publisher chatter_pub3 = n.advertise<beginner_tutorials::Int32Stamped>("distance3", 10);
	ros::Publisher chatter_pub4 = n.advertise<beginner_tutorials::Int32Stamped>("distance4", 10);
	ros::Publisher chatter_pub5 = n.advertise<beginner_tutorials::Int32Stamped>("distance5", 10);
	ros::Subscriber on_off_sub = n.subscribe<std_msgs::Bool>("/uwb_on_off", 10, on_off_cb);	

	ros::Rate loop_rate(500);

	while (ros::ok()) {
		if (f.is_open()) {
			if (f >> str) {
				if (str.find(":")==1) {
				stream1.clear();
				stream1.str("");
				stream1.str(str);
				stream1 >> testnode;
				}
				if (str.find("mm")==2 ||str.find("mm")==3 || str.find("mm")==4 || str.find("mm")==5) {
				stream1.clear();
				stream1.str("");
				stream1.str(str);
				stream1 >> distances[testnode];
				cout << "distance of anchor " << testnode << " is " << distances[testnode] << endl;
				}
				if (testnode == node_id[2]) {
					if (counts[testnode] != distances[testnode]){
						beginner_tutorials::Int32Stamped msg;
						msg.data=distances[testnode];
						msg.header.stamp=ros::Time::now();
				    	chatter_pub2.publish(msg);
					}
					counts[testnode] = distances[testnode];
				}
				if (testnode == node_id[3]) {
					if (counts[testnode] != distances[testnode]){
						beginner_tutorials::Int32Stamped msg;
						msg.data=distances[testnode];
						msg.header.stamp=ros::Time::now();
				    	chatter_pub1.publish(msg);
					}
					counts[testnode] = distances[testnode];
				}
				if (testnode == node_id[0]) {
					if (counts[testnode] != distances[testnode]){
						beginner_tutorials::Int32Stamped msg;
						msg.data=distances[testnode];
						msg.header.stamp=ros::Time::now();
				    	chatter_pub4.publish(msg);
					}
					counts[testnode] = distances[testnode];
				}
				if (testnode == node_id[4]) {
					if (counts[testnode] != distances[testnode]){
						beginner_tutorials::Int32Stamped msg;
						msg.data=distances[testnode];
						msg.header.stamp=ros::Time::now();
				    	chatter_pub5.publish(msg);
					}
					counts[testnode] = distances[testnode];
				}
				if (testnode == node_id[1]) {
					if (counts[testnode] != distances[testnode]){
						beginner_tutorials::Int32Stamped msg;
						msg.data=distances[testnode];
						msg.header.stamp=ros::Time::now();
				    	chatter_pub3.publish(msg);
					}
					counts[testnode] = distances[testnode];
				}
				
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
		if (f.is_open()){
			ROS_INFO("[UWB NODE]: Port already opened");
		} else {
			ROS_INFO("[UWB NODE]: Try opening port");			
			f.open(port_addr.c_str(), fstream::in);
			if (f.is_open()){
				ROS_INFO("[UWB NODE]: Port opened sucessfully");
			} else {
				ROS_INFO("[UWB NODE]: Error openning port");
			}
		}
	} else {
		if (f.is_open()){
			f.close();
			if (f.is_open()) ROS_INFO("[UWB NODE]: Error closing port");	
			else ROS_INFO("[UWB NODE]: Port closed sucessfully");
		} else {
			ROS_INFO("[UWB NODE]: Port already closed");
		}
	}
}