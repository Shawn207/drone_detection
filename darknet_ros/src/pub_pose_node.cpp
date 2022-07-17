#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
int main(int argc, char** argv) {
    ros::init(argc, argv, "darknet_ros_pub_pose");
    ros::NodeHandle nh;

    static tf2_ros::TransformBroadcaster broadcaster;
    geometry_msgs::TransformStamped tfs;
    tfs.header.frame_id = "world";
    tfs.header.stamp = ros::Time::now();
    tfs.child_frame_id = "camera";

    tfs.transform.translation.x = 0.0;
    tfs.transform.translation.y = 0.0;
    tfs.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0,0,0);
    tfs.transform.rotation.x = q.getX();
    tfs.transform.rotation.y = q.getY();
    tfs.transform.rotation.z = q.getZ();
    tfs.transform.rotation.w = q.getW();


    // mvoe forward while keeping a fixed quaternion. change pose evevry 3 s
    int i = -1;
    
    while(ros::ok())
    {
        i++;
        // q.setRPY(0,0,1.57*i);
        // tfs.transform.rotation.x = q.getX();
        // tfs.transform.rotation.y = q.getY();
        // tfs.transform.rotation.z = q.getZ();
        // tfs.transform.rotation.w = q.getW();
        // tfs.transform.translation.x = i*1.0;
        // tfs.transform.translation.y = i*1.0;
        // tfs.transform.translation.z = i*1.0;
        ros::Time start = ros::Time::now();
        ros::Duration time(18.0);
        while(ros::Time::now()-start <= time){
            tfs.header.stamp = ros::Time::now();
            // ROS_INFO("Check current time stamp!!");
            broadcaster.sendTransform(tfs);
        }
        // printf(
        //     "time out, i=%d",i
        // );
        if (i == 3)
        {
            i = -1;
        }
        // loop_rate.sleep();
    }


    ros::spin();
    return 0;
}
