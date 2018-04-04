#include "ros/ros.h"
#include <iostream>
#include <Eigen/Dense>
#include "math.h"
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Twist.h>
#include <beginner_tutorials/Int32Stamped.h>

using namespace Eigen;
using namespace std;

//for now position of anchors are (-300,-300),(-300,300),(300,300),(300,-300)
int g_anc[8]={-165,-250,-165,250,165,250,165,-250};
int g_scale=50;
int g_holdpos=0, g_timeout=0;
float g_norm=1000,g_xvec=0,g_yvec=0,g_distcheck=3000;
geometry_msgs::Twist g_twistmsg;

//Header for calculating delta t for Kalman filters
std_msgs::Header old1, now1, old2, now2, old3, now3, old4, now4;

//Initialize Matrices for Kalman filters
Matrix2f A1;
Matrix2f A2;
Matrix2f A3;
Matrix2f A4;

Matrix2f H;

Matrix2f R;

Matrix2f P1;
Matrix2f P2;
Matrix2f P3;
Matrix2f P4;

Matrix2f Q;

Matrix2f K1;
Matrix2f K2;
Matrix2f K3;
Matrix2f K4;

Matrix2f z1;
Matrix2f z2;
Matrix2f z3;
Matrix2f z4;

Matrix2f kpos1;
Matrix2f kpos2;
Matrix2f kpos3;
Matrix2f kpos4;

Matrix2f I;

Matrix2f temp1;
Matrix2f temp2;
Matrix2f temp3;
Matrix2f temp4;


//Global variables to get trilateration result
float g_x,g_y,g_z;

//Variables to check if 4 distances are received
int check[3]={0};

//array to store trilateration results from global variables
float pos[12]={0};

//average position
float finalpos[3]={};

//nan counter
int nanc;

//Pseudo Inverse Function
void pinv( Matrix2f& pinvmat)
{
    JacobiSVD<Matrix2f> svd(pinvmat,ComputeFullU|ComputeFullV); // Compute the SVD of pinvmat
    double  pinvtoler=1.e-6; // choose your tolerance widely!
    typename JacobiSVD<Matrix2f>::SingularValuesType sigma,sigma_inv;
    sigma  = svd.singularValues();
    sigma_inv.resizeLike(sigma);
    for ( int i=0; i<sigma.size();i++) {
      if ( sigma(i) > pinvtoler )
        sigma_inv(i)=1.0/sigma(i);
      else
        sigma_inv(i) = 0;
    }
    Matrix2f diag(pinvmat.cols(), pinvmat.rows());
    diag.setZero();
    diag.diagonal() = sigma_inv;
    pinvmat =  svd.matrixV()*diag*svd.matrixU().transpose() ;
}

//trilateration function
void trilateration(int a1,int a2, int b2, float d0,float d1, float d2){
    
  g_x=(pow(a1,2) - pow(d1,2) + pow(d0,2))/(2*a1) ;
  g_y=(pow(d0,2) - pow(d2,2) + pow(b2,2) + pow(a2,2))/(2*b2) - (a2/b2)*g_x;
  g_z= sqrt(pow(d1,2) - pow(g_x,2) - pow(g_y,2));
  }





void dist1(const beginner_tutorials::Int32Stamped::ConstPtr& msg)
{
  float delt, reading, corrected;

  reading = msg->data;
  now1 = msg->header;
  delt=now1.stamp.nsec/1000000 - old1.stamp.nsec/1000000 + now1.stamp.sec*1000 - old1.stamp.sec*1000;
  delt=delt/1000;
  old1 = now1;
  A1(0,1)=delt;
  cout << "raw = \t" << reading << "\n";
  corrected =0.97*reading + 411.551;
  cout << "corr = \t" << corrected << "\n";
  z1(0,0) = corrected;
  kpos1 = A1*kpos1;
  P1 = A1*P1*A1.transpose() + Q;
  temp1= H*P1*H.transpose()+R;
  pinv(temp1);
  K1 = P1*H.transpose()*temp1;
  kpos1 = kpos1 + K1*(z1 - H*kpos1);
  P1 = (I-K1*H)*P1;
  cout << "guess1 = \n" << kpos1 << "\n\n"; //<< "gain = \n" << K1 << "\n" <<"\n";
  check[0]=1;
  
  g_timeout++;
  if (g_timeout>15){
      g_twistmsg.linear.x=0;
      g_twistmsg.linear.y=0;
      g_twistmsg.angular.z=0;
  }
}

void dist2(const beginner_tutorials::Int32Stamped::ConstPtr& msg)
{
  float delt, reading, corrected;

  reading = msg->data;
  now2 = msg->header;
  delt=now2.stamp.nsec/1000000 - old2.stamp.nsec/1000000 + now2.stamp.sec*1000 - old2.stamp.sec*1000;
  delt=delt/1000;
  old2 = now2;
  A2(0,1)=delt;
  cout << "raw = \t" << reading << "\n";
  corrected = 0.97*reading + 411.551;
  cout << "corr = \t" << corrected << "\n";
  z2(0,0) = corrected;
  kpos2 = A2*kpos2;
  P2 = A2*P2*A2.transpose() + Q;
  temp2= H*P2*H.transpose()+R;
  pinv(temp2);
  K2 = P2*H.transpose()*temp2;
  kpos2 = kpos2 + K2*(z2 - H*kpos2);
  P2 = (I-K2*H)*P2;
  cout << "guess2 = \n" << kpos2 << "\n\n" ;//<< "gain = \n" << K2 << "\n" <<"\n";
  check[1]=1;

  g_timeout++;
  if (g_timeout>15){
      g_twistmsg.linear.x=0;
      g_twistmsg.linear.y=0;
      g_twistmsg.angular.z=0;
  }
}

void dist3(const beginner_tutorials::Int32Stamped::ConstPtr& msg)
{
  float delt, reading, corrected;

  reading = msg->data;
  now3 = msg->header;
  delt=now3.stamp.nsec/1000000 - old3.stamp.nsec/1000000 + now3.stamp.sec*1000 - old3.stamp.sec*1000;
  delt=delt/1000;
  old3 = now3;
  A3(0,1)=delt;
  cout << "raw = \t" << reading << "\n";
  corrected = 0.97*reading + 411.551;
  cout << "corr = \t" << corrected << "\n";
  z3(0,0) = corrected;
  kpos3 = A3*kpos3;
  P3 = A3*P3*A3.transpose() + Q;
  temp3= H*P3*H.transpose()+R;
  pinv(temp3);
  K3 = P3*H.transpose()*temp3;
  kpos3 = kpos3 + K3*(z3 - H*kpos3);
  P3 = (I-K3*H)*P3;
  cout << "guess3 = \n" << kpos3 << "\n\n" ;//<< "gain = \n" << K3 << "\n" <<"\n";
  check[2]=1;

  g_timeout++;
  if (g_timeout>15){
      g_twistmsg.linear.x=0;
      g_twistmsg.linear.y=0;
      g_twistmsg.angular.z=0;
  }
}
class SubscribeAndPublish
{
  public:
    SubscribeAndPublish()
    {
      //Topic you want to publish
      marker_pub_ = n_.advertise<visualization_msgs::Marker>("/markers", 100);
      
      sub4_ = n_.subscribe("/distance1", 1000, &SubscribeAndPublish::dist4, this);
    }

  void dist4(const beginner_tutorials::Int32Stamped::ConstPtr& msg)
  {
    float delt, reading, corrected;

    reading = msg->data;
    now4 = msg->header;
    delt=now4.stamp.nsec/1000000 - old4.stamp.nsec/1000000 + now4.stamp.sec*1000 - old4.stamp.sec*1000;
    delt=delt/1000;
    old4 = now4;
    A4(0,1)=delt;
    cout << "raw = \t" << reading << "\n";
    corrected = 0.97*reading + 411.551;
    cout << "corr = \t" << corrected << "\n";
    z4(0,0) = corrected;
    kpos4 = A4*kpos4;
    P4 = A4*P4*A4.transpose() + Q;
    temp4= H*P4*H.transpose()+R;
    pinv(temp4);
    K4 = P4*H.transpose()*temp4;
    kpos4 = kpos4 + K4*(z4 - H*kpos4);
    P4 = (I-K4*H)*P4;
    cout << "guess4 = \n" << kpos4 << "\n\n";// << "gain = \n" << K4 << "\n" <<"\n";

    g_timeout++;
    if (g_timeout>15){
      g_twistmsg.linear.x=0;
      g_twistmsg.linear.y=0;
      g_twistmsg.angular.z=0;
    }
    // check if 4 distances are complete
    if ((check[0]==1)&&(check[1]==1)&&(check[2]==1)){

      //getting each position with respect to center
      trilateration((g_anc[6]-g_anc[0]),(g_anc[4]-g_anc[0]),(g_anc[5]-g_anc[1]),kpos1(0,0),kpos4(0,0),kpos3(0,0));
      pos[0]=g_x+g_anc[0];
      pos[1]=g_y+g_anc[1];
      pos[2]=g_z;

      trilateration((g_anc[0]-g_anc[6]),(g_anc[2]-g_anc[6]),(g_anc[3]-g_anc[7]),kpos4(0,0),kpos1(0,0),kpos2(0,0));
      pos[3]=g_x+g_anc[6];
      pos[4]=g_y+g_anc[7];
      pos[5]=g_z;

      trilateration((g_anc[4]-g_anc[2]),(g_anc[6]-g_anc[2]),(g_anc[7]-g_anc[3]),kpos2(0,0),kpos3(0,0),kpos4(0,0));
      pos[6]=g_x+g_anc[2];
      pos[7]=g_y+g_anc[3];
      pos[8]=g_z;

      trilateration((g_anc[2]-g_anc[4]),(g_anc[0]-g_anc[4]),(g_anc[1]-g_anc[5]),kpos3(0,0),kpos2(0,0),kpos1(0,0));
      pos[9]=g_x+g_anc[4];
      pos[10]=g_y+g_anc[5];
      pos[11]=g_z;

      for (int j=0;j<12;j++){
        cout<<pos[j]<<"\n";
      }
      cout<<"\n";

      //Averaging of results
      nanc = isnan(pos[2])+isnan(pos[5])+isnan(pos[8])+isnan(pos[11]);
      //cout<<nanc<<"\n";
      if (nanc<4){
        g_timeout=0;
        for(int cnt=0; cnt<2; cnt++ ){
          //finalpos[cnt]=( pos[cnt]*(isnan(pos[2])-1) + pos[cnt+3]*(isnan(pos[5])-1) + pos[cnt+6]*(isnan(pos[8])-1) + pos[cnt+9]*(isnan(pos[11])-1) )/(nanc-4);
          finalpos[cnt]=(pos[cnt]+pos[cnt+3]+pos[cnt+6]+pos[cnt+9])/4;
        }
        for(int cnt2=2; cnt2<12; cnt2+=3){
          if (isnan(pos[cnt2])){

          }
          else{
          finalpos[2] += pos[cnt2];
          } 
        }
        finalpos[2] = finalpos[2]/(4-nanc);
        cout << "Position: ( " << finalpos[0] << " , " << finalpos[1] << " , "<< finalpos[2] << " )\n";
      
      static tf2_ros::TransformBroadcaster br;
      geometry_msgs::TransformStamped transform2;
      transform2.header.stamp = now4.stamp;
      transform2.header.frame_id = "world";
      transform2.child_frame_id = "ugv";
      transform2.transform.translation.x = finalpos[0]/1000;
      transform2.transform.translation.y = finalpos[1]/1000;
      transform2.transform.translation.z = finalpos[2]/1000;
      transform2.transform.rotation.x = 0;
      transform2.transform.rotation.y = 0;
      transform2.transform.rotation.z = 0;
      transform2.transform.rotation.w = 1;
      br.sendTransform(transform2);
      
      
      points.header.frame_id = line_strip.header.frame_id = "world";
      points.header.stamp = line_strip.header.stamp = now4.stamp;
      pointbank.action= points.action = line_strip.action = visualization_msgs::Marker::ADD;
      points.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
      points.id = 0;
      pointbank.id=2;
      line_strip.id = 1;
      points.type = visualization_msgs::Marker::POINTS;
      line_strip.type = visualization_msgs::Marker::LINE_STRIP;
      points.scale.x = 0.1;
      points.scale.y = 0.1;
      line_strip.scale.x = 0.05;
      //points are green
      points.color.g = 1.0f;
      points.color.a = 1.0;

      // Line strip is blue
      line_strip.color.b = 1.0;
      line_strip.color.a = 1.0;
      geometry_msgs::Point p;
      p.x = finalpos[0]/1000;
      p.y = finalpos[1]/1000;
      p.z = finalpos[2]/1000;
      cout<<p<<"\n";
      pointbank.points.push_back(p);
      points.points=pointbank.points;
      line_strip.points=pointbank.points;
      marker_pub_.publish(points);
      marker_pub_.publish(line_strip);
      
      
      g_distcheck=sqrt(pow(finalpos[0],2)+pow(finalpos[1],2));
      //g_holdpos++;
      //if (g_holdpos==10){
      g_norm = sqrt(pow(finalpos[0],2)+pow(finalpos[1],2));
      g_xvec = finalpos[0]/(g_norm*g_scale);
      g_yvec = finalpos[1]/(g_norm*g_scale);
      //g_holdpos=0;
      //}
      cout<<"Distance: "<<g_distcheck<<"\n";
      cout<<"Scale: "<<g_scale<<"\n";
      if(g_distcheck>600){
        g_twistmsg.linear.x=g_yvec;
        g_twistmsg.linear.y=-g_xvec;
        g_twistmsg.angular.z=0;
        
    	}
      else{
        g_twistmsg.linear.x=0;
        g_twistmsg.linear.y=0;
        g_twistmsg.angular.z=0;
        }
      }  

      finalpos[2]=0;
      check[0]=0;
      check[1]=0;
      check[2]=0;
      
    }


  }

  private:
    ros::NodeHandle n_; 
    ros::Publisher marker_pub_;
    ros::Subscriber sub4_;
    visualization_msgs::Marker points, pointbank, line_strip;
};  

int main(int argc, char **argv)
{
  A1 << 1 , 0.1 , 0 , 1;
  A2 << 1 , 0.1 , 0 , 1;
  A3 << 1 , 0.1 , 0 , 1;
  A4 << 1 , 0.1 , 0 , 1;

  H << 1 , 0 , 0 , 0;

  R << 1000 , 0 , 0 , 0;

  P1 << 400 , 0 , 0 , 200;
  P2 << 400 , 0 , 0 , 200;
  P3 << 400 , 0 , 0 , 200;
  P4 << 400 , 0 , 0 , 200;

  Q << 100 , 0 , 0 , 0.05;

  I << 1, 0 , 0 ,1;

  K1 << 0 , 0 , 0 , 0;
  K2 << 0 , 0 , 0 , 0;
  K3 << 0 , 0 , 0 , 0;
  K4 << 0 , 0 , 0 , 0;

  z1 << 0 , 0 , 0 , 0;
  z2 << 0 , 0 , 0 , 0;
  z3 << 0 , 0 , 0 , 0;
  z4 << 0 , 0 , 0 , 0;

  kpos1 << 0 , 0 , 0 , 0;
  kpos2 << 0 , 0 , 0 , 0;
  kpos3 << 0 , 0 , 0 , 0;
  kpos4 << 0 , 0 , 0 , 0;

  temp1 << 0 , 0 , 0 , 0;
  temp2 << 0 , 0 , 0 , 0;
  temp3 << 0 , 0 , 0 , 0;
  temp4 << 0 , 0 , 0 , 0;
  g_twistmsg.linear.x=0;
  g_twistmsg.linear.y=0;
  g_twistmsg.angular.z=0;

  
  ros::init(argc, argv, "localize");

  ros::NodeHandle n;
  ros::NodeHandle nh_param("~");
  nh_param.param<int>("scale",g_scale,g_scale);
  //ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  SubscribeAndPublish SAPObject;
  ros::Subscriber sub1 = n.subscribe("/distance2", 1000, dist1);
  ros::Subscriber sub2 = n.subscribe("/distance4", 1000, dist2);
  ros::Subscriber sub3 = n.subscribe("/distance5", 1000, dist3);
  //ros::Subscriber sub4 = n.subscribe("/distance1", 1000, dist4);
  ros::Publisher ugv_command_pub = n.advertise<geometry_msgs::Twist>("/ugv_base_controller/in/cmd_vel",100);
  ros::Rate loop_rate(6);
  while(ros::ok()){
  ugv_command_pub.publish(g_twistmsg);
  ros::spinOnce();
  loop_rate.sleep();
  }
  
  

  ros::spin();

  return 0;
}
