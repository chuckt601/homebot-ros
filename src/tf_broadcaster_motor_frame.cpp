#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_publisher");
  ros::NodeHandle n;

  ros::Rate r(100);

  tf::TransformBroadcaster broadcaster;

  while(n.ok()){
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0,0,-0.3420201,0.9396926), tf::Vector3(0.0, 0.0, 0.0)),
        ros::Time::now(),"camera_frame", "motor_frame"));
    r.sleep();
  }
}
