#include "ros/ros.h"
#include <sstream>
#include <fstream>
#include <string>
#include <iostream>
using namespace std;

ifstream inFile;
string data1;
int num1[500] = {0};
int cnt = 0;
int size;
float mean = 0;
float var =0;
int main(int argc, char **argv)
{
	inFile.open("/home/hans/catkin_ws/try.txt");
	if(!inFile)
	{
		cerr << "Unable to open file";
	}
	while (inFile >> data1)
	{
		cout << data1  << "\n";
		if (data1.compare(data1.size() - 3,1, ",")==0)
		{
			data1 = data1.substr(data1.size()-2);
		}
		else if (data1.compare(data1.size() - 4,1, ",")==0)
		{
			data1 = data1.substr(data1.size()-3);
		}
		else if (data1.compare(data1.size() - 5,1, ",")==0)
		{
			data1 = data1.substr(data1.size()-4);
		}
		else if (data1.compare(data1.size() - 6,1, ",")==0)
		{
			data1 = data1.substr(data1.size()-5);
		}
		else
		{
			cout << "not data";
			continue;
		}
		//cout <<data1 << "\n";
		stringstream cv(data1);
		cv >> num1[cnt];
		cout << num1[cnt] << "\n";
		cnt = cnt +1;
	}
	for (int i =0 ; i<500; i++)
	{
		mean = mean + num1[i];
		if (num1[i] == 0)
		{
			mean = mean / i;
			size =i;
			break;
		}
	}
	for (int i=0; i<size ; i++)
	{
		var = var + (num1[i] - mean)*(num1[i]-mean);
	}
	var = var/size;
	cout<< mean << "  " << var;
	return 0;
}