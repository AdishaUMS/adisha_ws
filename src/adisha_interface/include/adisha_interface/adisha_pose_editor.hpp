#ifndef __ADISHA_MOTION_SEQUENCER_HPP__
#define __ADISHA_MOTION_SEQUENCER_HPP__

#include <cstdlib>
#include <cstring>
#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <yaml-cpp/yaml.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "adisha_interface/msg/joint_torque.hpp"

#define ADISHA_POSE_EDITOR_CLEAR        "\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n"
#define ADISHA_POSE_YAML_PATH           "src/adisha_controller/config/adisha_pose.yaml"
#define ADISHA_ACTUATORS_INFO_YAML_PATH "src/adisha_controller/config/adisha_joints_info.yaml"
#define SPIN_RATE                       1000000/60



namespace adisha 
{



    struct pose_t {
        std::string                         name,
                                            desc;
        std::vector<std::vector<uint64_t>>  motion;
    };



    class PoseEditor : public rclcpp::Node {
    

    private:
        enum app_state_t{
            MAIN_MENU,
            EDIT_POSE,
            CREATE_POSE,
            CONFIG_POSE,
            DELETE_POSE
        };

        YAML::Node                      pose_yaml,
                                        actuators_info_yaml;

        pose_t                          pose_select;

        std::vector<std::string>        pose_list,
                                        joint_list,
                                        cmd_buf;

        std::vector<uint8_t>            joint_id;

        uint8_t                         joint_num;

        app_state_t                     app_state;

        char                            char_buf;

        std::string                     str_buf;

        rclcpp::TimerBase::SharedPtr    pose_editor_timer;

        rclcpp::Publisher<adisha_interface::msg::JointTorque>::SharedPtr joint_torque_pub;

        void printPoseList();
        void parseCommand();


    public:
        PoseEditor();
        ~PoseEditor();

        bool selectPose(const char* name);
        bool createNewPose(const char* name);
        void setPoseName(const char* name);
        void setPoseDesc(const char* desc);
        
        void torqueEnable(uint8_t joint_id);
        void torqueEnableSome(uint8_t* joint_id);
        void torqueEnableAll();
        void torqueDisable(uint8_t joint_id);
        void torqueDisableSome(uint8_t* joint_id);
        void torqueDisableAll();
        void updateJointVal();
        void savePose();
        void exitApp();

        void appTimerCallback();
    };
}



#endif