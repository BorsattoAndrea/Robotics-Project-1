#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "std_msgs/String.h"
#include "project_1/CustomOdometry.h"
#include <sstream>
#include <dynamic_reconfigure/server.h>
#include "project_1/integrationConfig.h"
#include "project_1/reset.h"
#include "project_1/reset_general.h"

class Pub_sub_odometry {

private:
    ros::NodeHandle n;
    ros::Subscriber sub;
    ros::Publisher odom_pub;
    ros::Publisher custom_pub;
    ros::ServiceServer resetZeroService;
    ros::ServiceServer resetGeneralService;

    ros::Time lastTime;
    double x,y,th;
    tf::TransformBroadcaster odom_broadcaster;

    int integrationType;


public:
    Pub_sub_odometry() {
        sub = n.subscribe("/twist", 1, &Pub_sub_odometry::computeOdometry, this);

        odom_pub = n.advertise<nav_msgs::Odometry>("/odometry", 1);
        custom_pub = n.advertise<project_1::CustomOdometry>("/custom", 1);
        resetZeroService = n.advertiseService("reset_zero" , &Pub_sub_odometry::resetZero, this);
        resetGeneralService = n.advertiseService("reset_general" , &Pub_sub_odometry::resetGeneral, this);
        lastTime = ros::Time::now();

        //Reads param from launch file
        n.getParam("/InitialX", x);
        n.getParam("/InitialY", y);
        n.getParam("/InitialTheta", th);
        integrationType = 0;
    }

    void computeOdometry(const geometry_msgs::TwistStamped::ConstPtr& msg){
        double vx, w, dt;
        ros::Time currentTime;

        //Reads currentTime from message's header
        currentTime = msg->header.stamp;

        //Computes dt from last message
        dt = (currentTime - lastTime).toSec();
        //Computes reads linear and angular velocities from message
        vx = msg->twist.linear.x;
        w = msg->twist.angular.z;

        //Integration
        if(integrationType == 0) { //EULER
            x += vx * cos(th) * dt;
            y += vx * sin(th) * dt;
            th += w * dt;
        }
        else if(integrationType == 1){ //RUNGE-KUTTA
            x += vx * cos(th + w * dt / 2) * dt;
            y += vx * sin(th + w * dt / 2) * dt;
            th += w * dt;
        }

        //Publish tf transformation
        publishTfTransformation(currentTime);
        //Publish odometry message
        publishOdometry(vx, w, currentTime);

        //Updates last time
        lastTime= currentTime;
    }

    void publishOdometry(double vx, double w, ros::Time currentTime){
        nav_msgs::Odometry odometry;
        geometry_msgs::Quaternion odometryQuaternion = tf::createQuaternionMsgFromYaw(th);
        project_1::CustomOdometry customOdometry;

        //set header
        odometry.header.stamp = currentTime;
        odometry.header.frame_id = "odom";
        //set pose
        odometry.pose.pose.position.x = x;
        odometry.pose.pose.position.y = y;
        odometry.pose.pose.position.z = 0.0;
        odometry.pose.pose.orientation = odometryQuaternion;
        //set velocity
        odometry.child_frame_id = "baseLink";
        odometry.twist.twist.linear.x = vx;
        odometry.twist.twist.linear.y = 0;
        odometry.twist.twist.angular.z = w;

        //publish custom odometry
        customOdometry.odom = odometry;
        if(integrationType==0)
            customOdometry.method.data = "euler";
        else
            customOdometry.method.data = "rk";

        custom_pub.publish(customOdometry);

        //publish odometry
        odom_pub.publish(odometry);
    }

    void publishTfTransformation(ros::Time currentTime){
        geometry_msgs::TransformStamped odometryTransformation;
        geometry_msgs::Quaternion odometryQuaternion = tf::createQuaternionMsgFromYaw(th);

        //set header
        odometryTransformation.header.stamp = currentTime;
        odometryTransformation.header.frame_id = "odom";
        odometryTransformation.child_frame_id = "base_link";
        //set transformation
        odometryTransformation.transform.translation.x = x;
        odometryTransformation.transform.translation.y = y;
        odometryTransformation.transform.translation.z = 0;
        odometryTransformation.transform.rotation = odometryQuaternion;

        //publish transformation
        odom_broadcaster.sendTransform(odometryTransformation);
    }

    void setIntegration(project_1::integrationConfig &config){
        integrationType = config.integration;
    }

    bool resetZero(project_1::reset::Request  &req,
                   project_1::reset::Response &res)
    {
        x = 0;
        y = 0;
        ROS_INFO("x reset to: %f", x);
        ROS_INFO("y reset to: %f", y);
        return true;
    }

    bool resetGeneral(project_1::reset_general::Request  &req,
                      project_1::reset_general::Response &res)
    {
        x = req.x;
        y = req.y;
        th = req.th;
        ROS_INFO("x reset to: %f", x);
        ROS_INFO("y reset to: %f", y);
        ROS_INFO("th reset to: %f", th);
        return true;
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "AgileXCore");
    Pub_sub_odometry pubSubOdometry;

    dynamic_reconfigure::Server<project_1::integrationConfig> server;
    dynamic_reconfigure::Server<project_1::integrationConfig>::CallbackType f;

    f = boost::bind(&Pub_sub_odometry::setIntegration, &pubSubOdometry, _1);
    server.setCallback(f);

    ros::spin();
    return 0;
}