#include "ros/ros.h"
#include "robotics_hw1/MotorSpeed.h"
#include "project_1/Speed.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#define RPM_IN_RADIANS 0.10472
#define WHEEL_RADIUS 0.1575
#define GIVEN_GEAR_RATIO 1/35
#define COMPUTED_GEAR_RATIO 1/38.2 //0.026122942759

class LateralWheelSync
{
private:
    ros::NodeHandle n;

    message_filters::Subscriber<robotics_hw1::MotorSpeed> sub1;
    message_filters::Subscriber<robotics_hw1::MotorSpeed> sub2;
    ros::Publisher pub;
    typedef message_filters::sync_policies::ApproximateTime<robotics_hw1::MotorSpeed, robotics_hw1::MotorSpeed> MySyncPolicy;
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    boost::shared_ptr<Sync> sync;

public:
    LateralWheelSync(int reverse){
        sub1.subscribe(n, "/motor_speed_fl", 1);
        sub2.subscribe(n, "/motor_speed_rl", 1);
        pub = n.advertise<project_1::Speed>("/syncVelocity_l", 1);
        sync.reset(new Sync(MySyncPolicy(10), sub1, sub2));

        if(reverse==1)
            sync->registerCallback(boost::bind(&LateralWheelSync::syncCallbackReverse, this, _1, _2));
        else
            sync->registerCallback(boost::bind(&LateralWheelSync::syncCallback, this, _1, _2));
    }

    void syncCallback(const robotics_hw1::MotorSpeed::ConstPtr& msg1,
                  const robotics_hw1::MotorSpeed::ConstPtr& msg2) {
        ROS_INFO ("syncCallback");
        project_1::Speed speed;
        speed.header = msg1->header;
        speed.metersXSecond = (msg1->rpm + msg2->rpm) * RPM_IN_RADIANS * COMPUTED_GEAR_RATIO * WHEEL_RADIUS / 2;
        pub.publish(speed);
    }

    void syncCallbackReverse(const robotics_hw1::MotorSpeed::ConstPtr& msg1,
                      const robotics_hw1::MotorSpeed::ConstPtr& msg2) {
        ROS_INFO ("syncCallback");
        project_1::Speed speed;
        speed.header = msg1->header;
        speed.metersXSecond = (-msg1->rpm - msg2->rpm) * RPM_IN_RADIANS * COMPUTED_GEAR_RATIO * WHEEL_RADIUS / 2;
        pub.publish(speed);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Synchronizer");
    int reverse = atoi(argv[1]);
    LateralWheelSync sync(reverse);
    ros::spin();
    return 0;
}


