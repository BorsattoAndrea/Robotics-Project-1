#include "ros/ros.h"
#include "project_1/Speed.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Float64.h"
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>

class BaselineCalculator
{
private:
    ros::NodeHandle n;
    double gearRatioSum;
    int count;
    message_filters::Subscriber<project_1::Speed> sub1;
    message_filters::Subscriber<project_1::Speed> sub2;
    message_filters::Subscriber<nav_msgs::Odometry> sub3;
    ros::Publisher pub;
    typedef message_filters::sync_policies::ApproximateTime<project_1::Speed, project_1::Speed, nav_msgs::Odometry> MySyncPolicy;
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    boost::shared_ptr<Sync> sync;

public:
    BaselineCalculator(){
        sub1.subscribe(n, "/syncVelocity_l", 1);
        sub2.subscribe(n, "/syncVelocity_r", 1);
        sub3.subscribe(n, "/scout_odom", 1);
        pub = n.advertise<std_msgs::Float64>("/baseline", 1);
        sync.reset(new Sync(MySyncPolicy(10), sub1, sub2, sub3));
        sync->registerCallback(boost::bind(&BaselineCalculator::syncCallback, this, _1, _2, _3));
        count = 0;
        gearRatioSum = 0;
    }

    void syncCallback(const project_1::Speed::ConstPtr& velL, const project_1::Speed::ConstPtr& velR, const nav_msgs::Odometry::ConstPtr& odom) {
        ROS_INFO ("processing baseline calculation");
        std_msgs::Float64 result;

        double angularVelocity = odom->twist.twist.angular.z;
        double speedR = velR->metersXSecond;
        double speedL = velL->metersXSecond;

        if(angularVelocity>0.005) {
            gearRatioSum += (speedR - speedL) / angularVelocity;
            count++;
        }

        if(count > 0) {
            result.data = gearRatioSum / count;
            pub.publish(result);
        }
    }

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "BaselineCalculator");
    BaselineCalculator sync;
    ros::spin();
    return 0;
}