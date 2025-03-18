#include "ros/ros.h"
#include "z1_subscriber/unitree_arm_sdk/control/unitreeArm.h"
#include "std_msgs/Float64.h"
#include "realsense_mediapipe/KeypointMsg.h"

using namespace UNITREE_ARM;
class Z1ARM : public unitreeArm {
    public:
        Z1ARM(): unitreeArm(true) {
            // Initialize publisher for gripper position (if needed)
            gripper_pub = nh.advertise<std_msgs::Float64>("/gripper_position", 1);
        };
        ~Z1ARM() {};
    
        void armCtrlByFSM();
        void armCtrlInJointCtrl();
        void armCtrlInCartesian();
        void printState();
    
        // ROS node handle and publishers/subscribers
        ros::NodeHandle nh;
        ros::Publisher gripper_pub;
    
        double gripper_pos = 0.0;
        double joint_speed = 2.0;
        double cartesian_speed = 0.5;
    };
    
    void Z1ARM::armCtrlByFSM() {
        Vec6 posture[2];
        gripper_pos = 0.0;
    
        ROS_INFO("[TO STATE]");
        labelRun("forward");
    
        ROS_INFO("[MOVEJ]");
        posture[0] << 0, 0, 1.57, 0, 0, 0.60;
        joint_speed = 2.0;
        MoveJ(posture[0], gripper_pos, joint_speed);
    
        // Publish gripper position as an example
        std_msgs::Float64 gripper_msg;
        gripper_msg.data = gripper_pos;
        gripper_pub.publish(gripper_msg);
    }
    
    void Z1ARM::armCtrlInJointCtrl() {
        labelRun("forward");
        startTrack(ArmFSMState::JOINTCTRL);
    
        Timer timer(_ctrlComp->dt);
        for (int i = 0; i < 1000; i++) {
            directions << 0, 0, 0, -1, 0, 0, -1;
            joint_speed = 1.0;
            jointCtrlCmd(directions, joint_speed);
            timer.sleep();
        }
    }
    
    void Z1ARM::armCtrlInCartesian() {
        labelRun("forward");
        startTrack(ArmFSMState::CARTESIAN);
    
        double angular_vel = 0.3;
        double linear_vel = 0.3;
        Timer timer(_ctrlComp->dt);
        for (int i = 0; i < 2000; i++) {
            directions << 0, 0, 0, 0, 0, 1, 0;
            cartesianCtrlCmd(directions, angular_vel, linear_vel);
            timer.sleep();
        }
    }
    
    void Z1ARM::printState() {
        ROS_INFO("------ joint State ------");
        ROS_INFO_STREAM("qState: " << lowstate->getQ().transpose());
        ROS_INFO_STREAM("qdState: " << lowstate->getQd().transpose());
        ROS_INFO_STREAM("tauState: " << lowstate->getTau().transpose());
    
        ROS_INFO("------ Endeffector Cartesian Posture ------");
        ROS_INFO("roll pitch yaw x y z");
        ROS_INFO_STREAM(lowstate->endPosture.transpose());
    }
    
    int main(int argc, char** argv) {
        // Initialize the ROS system
        ros::init(argc, argv, "z1_arm_control");
        ros::NodeHandle nh;
    
        Z1ARM arm;
        arm.sendRecvThread->start();
    
        arm.backToStart();
    
        // Control loop for the arm
        ros::Rate loop_rate(10);  // Loop rate (10 Hz)
        
        arm.armCtrlInCartesian();
        arm.printState();
    
        ros::spinOnce();
        loop_rate.sleep();
    
        
        return 0;
    }