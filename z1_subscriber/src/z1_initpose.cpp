#include "ros/ros.h"
#include "z1_subscriber/unitree_arm_sdk/control/unitreeArm.h"
#include "z1_subscriber/Z1Init.h"

using namespace UNITREE_ARM;
class Z1ARM : public unitreeArm{
    public:
        Z1ARM(): unitreeArm(true){
            ros::NodeHandle nh;
            
            // 订阅来自 Realsense 的鼻尖坐标信号
            // sub_keypoint = nh.subscribe("keypoint", 1000, &Z1ARM::keypointCallback, this);
            // 给 Realsense 发布 Z1 移动完成信号
            pub_move_complete = nh.advertise<z1_subscriber::Z1Init>("z1_init_complete", 10);
            ROS_INFO("Z1 Node Initialized");
            
        }
        ~Z1ARM() {
            backToStart();
        };
        void z1init(){
            init_high = 0.3;
            T_z = init_high + 0.28;
            Vec6 posture[2];
            
            posture[0] << 0, 0, 0, 0.0, 0.0, init_high;
            ROS_INFO("init_high=%f",init_high);
            joint_speed = 0.5;

            // 默认认为运动成功
            bool move_success = true;

            ROS_INFO("[TO STATE]");
            labelRun("forward");
            ROS_INFO("moved!");
            try {
                ROS_INFO("Attempting to move!");
                MoveL(posture[0], joint_speed);  // MoveJ movement command
                ROS_INFO("roll pitch yaw x y z");
                ROS_INFO_STREAM(lowstate->endPosture.transpose());
                ROS_INFO("Moved successfully!");
            } catch (const std::exception& e) {
                // If an error occurs (e.g., no inverse kinematics solution), log it
                ROS_ERROR("MoveJ failed: %s", e.what());
                // Here you could add logic to retry or skip to the next step
                // For example, you could just return and let the system try again later
                move_success = false;
                return;
            }
            
            init_result.move_success = move_success;
            init_result.T_z = T_z;
            pub_move_complete.publish(init_result);
            ROS_INFO("Move status published: %s", move_success ? "True" : "False");

        }


    private:
        ros::Subscriber sub_keypoint;
        ros::Publisher pub_move_complete;
        z1_subscriber::Z1Init init_result;
        double joint_speed = 1.0;
        double init_high = 0.3;
        float T_z = 0.28;

        
};

int main(int argc, char* argv[]){
    ros::init(argc, argv, "Z1");
    Z1ARM z1;
    z1.sendRecvThread->start();
    z1.backToStart();
    z1.z1init();
    ros::spin();
    return 0;

}