#include "ros/ros.h"
#include "project_1/Speed.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Float64.h"
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>

#define GIVEN_GEAR_RATIO 0.028571428
#define COMPUTED_GEAR_RATIO 0.026122942759 // = 1/38.2


class BaselineCalculator {
private:
    ros::NodeHandle n;
    double gearRatioSum;
    int count;

    message_filters::Subscriber <nav_msgs::Odometry> sub1;
    message_filters::Subscriber <nav_msgs::Odometry> sub3;
    ros::Publisher pub;

    typedef message_filters::sync_policies::ApproximateTime <nav_msgs::Odometry, nav_msgs::Odometry> MySyncPolicy;
    typedef message_filters::Synchronizer <MySyncPolicy> Sync;
    boost::shared_ptr <Sync> sync;

public:
    BaselineCalculator() {
        sub1.subscribe(n, "/odometry", 1);
        sub3.subscribe(n, "/scout_odom", 1);
        pub = n.advertise<std_msgs::Float64>("/gearRatio", 1);
        sync.reset(new Sync(MySyncPolicy(10), sub1, sub3));
        sync->registerCallback(boost::bind(&BaselineCalculator::syncCallback, this, _1, _2));

        count = 0;
        gearRatioSum = 0;
    }

    void syncCallback(const nav_msgs::Odometry::ConstPtr &myOdom,
                      const nav_msgs::Odometry::ConstPtr &scoutOdom) {
        ROS_INFO("processing gearRatio calculation");
        std_msgs::Float64 result;

        double linearVelocity = scoutOdom->twist.twist.linear.x;
        double computedLinearVelocity = myOdom->twist.twist.linear.x / GIVEN_GEAR_RATIO; //Now is COMPUTED_GEAR_RATIO

        //ROS_INFO("%lf, %lf, %lf", linearVelocity, speedL, speedR);
        ROS_INFO("%lf, %lf", linearVelocity, myOdom->twist.twist.linear.x);

        if (linearVelocity > 0) {
            gearRatioSum += linearVelocity / computedLinearVelocity;
            count++;
        }

        if (count > 0) {
            result.data = gearRatioSum / count;
            pub.publish(result);
        }
    }

};

int main(int argc, char **argv) {
    ros::init(argc, argv, "BaselineCalculator");
    BaselineCalculator sync;
    ros::spin();
    return 0;
}
