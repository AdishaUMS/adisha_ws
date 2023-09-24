#include "adisha_interface/adisha_pose_editor.hpp"



int main(int argc, char** argv) {
    /* Initiate rclcpp */
    rclcpp::init(argc, argv);

    /* Declare the node */
    auto pose_editor = std::make_shared<adisha::PoseEditor>();

    /* Spin the node */
    rclcpp::spin(pose_editor);

    /* Close */
    rclcpp::shutdown();
    return 0;
}