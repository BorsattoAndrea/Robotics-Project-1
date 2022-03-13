#include "ros/ros.h"
#include "project_1/Speed.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Twist.h"
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
// BASELINE 0.583  // real baseline
// BASELINE 1.16125063068 // first attempt considering all the angular velocities over 0 rad/s (before gear ratio calculation)
// BASELINE 1.129 // second attempt considering only angular velocities over 0.005 rad/s (before gear ratio calculation)

#define BASELINE 1.0467159947 // final baseline after GR

class GlobalSync
{
private:
    ros::NodeHandle n;

    message_filters::Subscriber<project_1::Speed> sub1;
    message_filters::Subscriber<project_1::Speed> sub2;
    ros::Publisher pub;
    typedef message_filters::sync_policies::ApproximateTime<project_1::Speed, project_1::Speed> MySyncPolicy;
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    boost::shared_ptr<Sync> sync;

public:
    GlobalSync(){
        sub1.subscribe(n, "/syncVelocity_l", 1);
        sub2.subscribe(n, "/syncVelocity_r", 1);
        pub = n.advertise<geometry_msgs::TwistStamped>("/twist", 1);
        sync.reset(new Sync(MySyncPolicy(10), sub1, sub2));
        sync->registerCallback(boost::bind(&GlobalSync::syncCallback, this, _1, _2));
    }

    void syncCallback(const project_1::Speed::ConstPtr& velL, const project_1::Speed::ConstPtr& velR) {
        ROS_INFO ("Global syncCallback");
        geometry_msgs::TwistStamped result;
        result.header = velL->header;

        geometry_msgs::Twist value;
        value.linear.y=0;
        value.linear.z=0;
        value.angular.x=0;
        value.angular.y=0;

        value.linear.x = (velR->metersXSecond + velL->metersXSecond)/2;
        value.angular.z = (velR->metersXSecond - velL->metersXSecond)/BASELINE;

        result.twist = value;

        pub.publish(result);
    }

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "GlobalSynchronizer");
    GlobalSync sync;
    ros::spin();
    return 0;
}