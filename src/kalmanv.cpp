
// %Tag(FULLTEXT)%
#include "ros/ros.h"
#include <beginner_tutorials/Int32Stamped.h>
#include <iostream>
#include <Eigen/Dense>
#include <std_msgs/Float32.h>

using namespace Eigen;
using namespace std;


float reading, corrected;
std_msgs::Float32 kalman,corr, raw;
float delt=0.1;
std_msgs::Header old, now;

Matrix2f A;

Matrix2f H;

Matrix2f R;

Matrix2f P;

Matrix2f Q;

Matrix2f K;
Matrix2f z;
Matrix2f kpos;
Matrix2f I;
Matrix2f temp;




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

class SubscribeAndPublish
{
  public:
    SubscribeAndPublish()
    {
      //Topic you want to publish
     kalman_pub_ = n_.advertise<std_msgs::Float32>("/kalman", 100);
     corrected_pub_ = n_.advertise<std_msgs::Float32>("/corrected", 100);
     raw_pub_ = n_.advertise<std_msgs::Float32>("/raw", 100);

      
      sub_ = n_.subscribe("/distance2", 1000, &SubscribeAndPublish::kalmanC, this);
    }

  void kalmanC(const beginner_tutorials::Int32Stamped::ConstPtr& msg)
  {
    //ROS_INFO("I heard: [%s]", msg->data.c_str());

    reading = msg->data;
    now = msg->header;
    delt=now.stamp.nsec/1000000 - old.stamp.nsec/1000000 + now.stamp.sec*1000 - old.stamp.sec*1000;
    delt=delt/1000;
    old = now;
    A(0,1)=delt;
    cout << "raw = \t" << reading << "\n";
    corrected = 0.97*reading + 411.551;
    cout << "corr = \t" << corrected << "\n";
    z(0,0) = corrected;
    kpos = A*kpos;
    P = A*P*A.transpose() + Q;
    temp= H*P*H.transpose()+R;
    pinv(temp);
    K = P*H.transpose()*temp;
    kpos = kpos + K*(z - H*kpos);
    P = (I-K*H)*P;
    cout << "guess = \n" << kpos << "\n" << "gain = \n" << K << "\n" <<"\n";
    raw.data=reading;
    corr.data=corrected;
    kalman.data=kpos(0,0);
    corrected_pub_.publish(corr);
    kalman_pub_.publish(kalman);
    raw_pub_.publish(raw);

    

  }
  private:
      ros::NodeHandle n_; 
      ros::Publisher corrected_pub_;
      ros::Publisher kalman_pub_;
      ros::Publisher raw_pub_;
      ros::Subscriber sub_;
};  


int main(int argc, char **argv)
{
  A << 1 , delt , 0 , 1;
  H << 1 , 0 , 0 , 0;
  R << 1000 , 0 , 0 , 0;
  P << 400 , 0 , 0 , 200;
  Q << 100 , 0 , 0 , 0.05;
  I << 1, 0 , 0 ,1;
  K << 0 , 0 , 0 , 0;
  z << 0 , 0 , 0 , 0;
  kpos << 0 , 0 , 0 , 0;
  temp << 0 , 0 , 0 , 0;

  ros::init(argc, argv, "listener");

  //ros::NodeHandle n;
  SubscribeAndPublish SAPObject;
  //ros::Subscriber sub = n.subscribe("/distance2", 1000, chatterCallback);

  ros::spin();
// %EndTag(SPIN)%

  return 0;
}

