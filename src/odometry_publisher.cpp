#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
//#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
//#include <sensor_msgs/ChannelFloat32>
#include <iostream>
#include <fstream>
#include <string>
#include <stdio.h>      
#include <stdlib.h> 
#include <robot_setup_tf/homebotToPi.h>
using namespace std;
double x;
double y;
double th;


double vx = 0.0;
double vy = -0.0;
double vth = 0.0;

void homebotToPiCallback(const robot_setup_tf::homebotToPi::ConstPtr& msg)
{
  //ROS_INFO("I heard: [%s]", msg->data); 
  x=msg->integrated_X;
  y=msg->integrated_Y;
  th=msg->yaw_IMU;
  vx=msg->X_rate;
  vy=msg->Y_rate;
}

class getdataFromAndroid {
	double x;
	double y;
	double th;
	double dx;
	double dy;
	double dth;
	double collectionTime;
	double lastCollectionTime;
  public:
    double getx() {return -x;}
    double gety() {return y;}
    double getth() {return th;}
    double getdx() {return dx;}
    double getdy() {return dy;}
    double getdth() {return dth;}
    double getTime() {return collectionTime;}
    void readAndroidFile();
};


int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");

  ros::NodeHandle n;
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  ros::Subscriber sub = n.subscribe("homebot_data", 1000, homebotToPiCallback);
  tf::TransformBroadcaster odom_broadcaster;
  //tf::TransformListener listener;
  //getdataFromAndroid android; //constructor
  //double x = 0.0;
  //double y = 0.0;
  //double th = 0.0;

  //double vx = 0;//0.1;
  //double vy = 0;//-0.1;
  //double vth = 0;//0.1;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ROS_INFO_STREAM("starting odom publisher") ;  

  ros::Rate r(50);
  while(n.ok()){

    ros::spinOnce();               // check for incoming messages
    //tf::StampedTransform transform;
    /*try{
      listener.lookupTransform("/map", "/odom",  
                               ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
    */
    
    
    
    current_time = ros::Time::now();

    //compute odometry in a typical way given the velocities of the robot
    //double dt = (current_time - last_time).toSec();
    //double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    //double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    //double delta_th = vth * dt;

    //x += delta_x;
    //x=transform.getOrigin() .x();
    //y=0;//transform.getOrigin() .y();
    //y += delta_y;
    //th += delta_th;
    //th=0;
    
    /*
    android.readAndroidFile();
    x=android.getx();
    y=android.gety();
    th=android.getth();
    vx=android.getdx();
    vy=android.getdy();
    vth=android.getdth();
    */
     
    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(-th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    


    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;
    r.sleep();
  }
}



void getdataFromAndroid::readAndroidFile(){
 //string lastLine;
 //int lastLineLen=0;
 std::string lastLine;
	
 //x=100.0;
 //y=50.0;
 //th=11.0;
 //dx=.1;
 //dx=.1;
 //dy=0;
 //dth=.1;
 //collectionTime=0;
 ifstream myfile;
 ROS_DEBUG_STREAM ("trying to open file");
 myfile.open ("/home/ubuntu/homebot/java/homebot/odometry.dat");
 if (myfile.is_open())
  {
    char ch;
    myfile.seekg(-1, std::ios::end);        // move to location 65 
    myfile.get(ch);                         // get next char at loc 66
    if (ch == '\n')
    {
      myfile.seekg(-2, std::ios::cur);    // move to loc 64 for get() to read loc 65 
      myfile.seekg(-1, std::ios::cur);    // move to loc 63 to avoid reading loc 65
      myfile.get(ch);                     // get the char at loc 64 ('5')
      while(ch != '\n')                   // read each char backward till the next '\n'
      {
         myfile.seekg(-2, std::ios::cur);    
         myfile.get(ch);
      }    
      std::getline(myfile,lastLine);
      //cout << "The last line : " << lastLine << '\n'; 
      ROS_INFO_STREAM( lastLine << "= the last line from the Arduino") ;    
    }
    else ROS_ERROR_STREAM ("odometry.dat not std CSV format");
  } 
  else ROS_ERROR_STREAM ("Unable to open odometry.dat");
   
 myfile.close();
 if(lastLine.length()>0){
  //std::string orbits ("686.97 365.24");
  std::string::size_type sz;     // alias of size_t
  std::string::size_type stringPointer; 
  collectionTime = std::stod (lastLine,&sz);
  if ( collectionTime!=lastCollectionTime){
	lastCollectionTime=collectionTime; 
	stringPointer=sz; 
    x = std::stod (lastLine.substr(stringPointer),&sz);    
    ROS_DEBUG_STREAM("x is being set to = "<< x) ; 
    ROS_DEBUG_STREAM("sz is = "<< sz) ;
    stringPointer+=sz;
    y = std::stod (lastLine.substr(stringPointer),&sz);
    stringPointer+=sz;
    ROS_DEBUG_STREAM("y is being set to = "<< y) ;
    ROS_DEBUG_STREAM("sz is = "<< sz) ;
    th = std::stod (lastLine.substr(stringPointer),&sz);
    ROS_DEBUG_STREAM("yaw is being set to = "<< th) ;
    ROS_DEBUG_STREAM("sz is = "<< sz) ;
    stringPointer+=sz;
    dx = std::stod (lastLine.substr(stringPointer),&sz);
    stringPointer+=sz;
    dy = std::stod (lastLine.substr(stringPointer),&sz);
    stringPointer+=sz;
    dth = std::stod (lastLine.substr(stringPointer),&sz);
  }    
 }
}
