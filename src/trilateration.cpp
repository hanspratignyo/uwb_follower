#include "ros/ros.h"
#include "beginner_tutorials/Int32Stamped.h"
#include "math.h"

using namespace std;

float g_x,g_y,g_z;
float pospoints[12] = {};
float g_a1= 400;
float g_a2= 400;
float g_b2= 400;
float g_b3= 400;

float error(int st, float d1,float d2,float d3,float d4){
	float err,dd1,dd2,dd3,dd4;
	dd1= sqrt(pow(pospoints[st],2)+pow(pospoints[st+1],2)+pow(pospoints[st+2],2));
	dd2= sqrt(pow(pospoints[st],2)+pow((pospoints[st+1]-g_b3),2)+pow(pospoints[st+2],2));
	dd3= sqrt(pow((pospoints[st]-g_a1),2)+pow(pospoints[st+1],2)+pow(pospoints[st+2],2));
	dd4= sqrt(pow((pospoints[st]-g_a2),2)+pow((pospoints[st+1]-g_b2),2)+pow(pospoints[st+2],2));
	cout<<dd1<<"\n"<<dd2<<"\n"<<dd3<<"\n"<<dd4<<"\n\n";
	err = pow((d1-dd1),2) + pow((d2-dd1),2) + pow((d3-dd3),2) + pow((d4-dd4),2);
	return err;
}

float trilateration(int a1,int a2, int b2, float d0,float d1, float d2){
		
	g_x=(pow(a1,2) - pow(d1,2) + pow(d0,2))/(2*a1) ;
	g_y=(pow(d0,2) - pow(d2,2) + pow(b2,2) + pow(a2,2))/(2*b2) - (a2/b2)*g_x;
	g_z= sqrt(pow(d1,2) - pow(g_x,2) - pow(g_y,2));
	}

int main(int argc, char **argv)
{
	//point 1,2,3,4 at (0,0) , (0,b3) , (a1,0) , (a2,b2)
	float d1,d2,d3,d4, a1,a2,b2,b3 ,test1,test2;
	float troll;
	troll= sqrt(-1);
	cout<< (isnan(troll)+isnan(troll))*30<<"\n";
	a1= 400;
	a2= 400;
	b2= 400;
	b3= 400;
	d1= 3098.08;
	d2= 2736.35;
	d3= 3042.13;
	d4= 2771.21;
	trilateration(a1,a2,b2,d1,d3,d4);
	pospoints[0]=g_x;
	pospoints[1]=g_y;
	pospoints[2]=g_z;
	cout << g_x << "\n" <<  g_y << "\n" <<  g_z << "\n" ; 
	cout<<"\n";
	cout<<error(0,d1,d2,d3,d4)<<"\n\n";

	trilateration(-a1,-a1,b3,d3,d1,d2);
	pospoints[3]=g_x+a1;
	pospoints[4]=g_y;
	pospoints[5]=g_z;
	cout << g_x << "\n" <<  g_y << "\n" <<  g_z << "\n" ; 
	test2 = sqrt(pow(pospoints[3],2)+pow(pospoints[4],2)+pow(pospoints[5],2));
	cout<<"\n";
	cout<<error(3,d1,d2,d3,d4)<<"\n\n";

	pospoints[6]=(pospoints[0]+pospoints[3])/2;
	pospoints[7]=(pospoints[1]+pospoints[4])/2;
	pospoints[8]=(pospoints[2]+pospoints[5])/2;
	cout<<error(6,d1,d2,d3,d4)<<"\n";

	trilateration(a2,-b3,a1,6202.,6202,5803);
	pospoints[6]=g_x;
	pospoints[7]=g_y+400;
	pospoints[8]=g_z;
	trilateration(-a2,-a2,-b2,6202.,6202,5803);
	pospoints[9]=g_x+400;
	pospoints[10]=g_y+400;
	pospoints[11]=g_z;
	



	return 0;
}

