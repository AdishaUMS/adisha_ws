#include "adisha_interface/adisha_pose_editor.hpp"



adisha::PoseEditor::PoseEditor() : Node("default") {
    /* Read .yaml files */
    this->pose_yaml             = YAML::LoadFile(ADISHA_POSE_YAML_PATH);
    this->actuators_info_yaml   = YAML::LoadFile(ADISHA_ACTUATORS_INFO_YAML_PATH);


    /* Get pose list from adisha_pose.yaml */
    this->pose_list = this->pose_yaml["pose_list"].as<std::vector<std::string>>();


    /* Get joint list */
    this->joint_list    = this->actuators_info_yaml["joint_list"].as<std::vector<std::string>>();
    this->joint_id      = this->actuators_info_yaml["joint_id"].as<std::vector<uint8_t>>();


    /* Get joint num */
    this->joint_num = (uint8_t)this->actuators_info_yaml["joint_num"].as<int>();


    /* App initialization */
    this->app_state = adisha::PoseEditor::MAIN_MENU;


    /* Create ROS2 instances */
    this->pose_editor_timer = this->create_wall_timer(std::chrono::microseconds(SPIN_RATE), std::bind(&adisha::PoseEditor::appTimerCallback, this));
}



adisha::PoseEditor::~PoseEditor() {
}



bool adisha::PoseEditor::selectPose(const char* name) {
    /* If name exists, select pose and return true */
    if(this->pose_yaml[name]) {
        this->setPoseName(name);
        this->setPoseDesc(this->pose_yaml[name]["desc"].as<std::string>().c_str());
        this->pose_select.motion = this->pose_yaml[name]["motion"].as<std::vector<std::vector<uint64_t>>>();
        return true;
    }

    /* Else, false */
    else {
        return false;
    }
}



bool adisha::PoseEditor::createNewPose(const char* name) {
    /* If name already exists, return false */
    if(this->pose_yaml[name]) {
        return false;
    }

    /* Else, create a new pose and return true */
    else {
        this->setPoseName(name);
        std::vector<uint64_t> zero  = {0, 0, 0};
        for(uint8_t i = 0; i < this->joint_num; ++i) {
            this->pose_select.motion.push_back(zero);
        }
        return true;
    }
}



void adisha::PoseEditor::setPoseName(const char* name) {
    this->pose_select.name = (std::string)name;
}



void adisha::PoseEditor::setPoseDesc(const char* desc) {
    this->pose_select.desc = (std::string)desc;
}



void adisha::PoseEditor::torqueEnable(uint8_t joint_id) {

}



void adisha::PoseEditor::torqueEnableSome(uint8_t* joint_id) {

}



void adisha::PoseEditor::torqueEnableAll() {

}



void adisha::PoseEditor::torqueDisable(uint8_t joint_id) {

}



void adisha::PoseEditor::torqueDisableSome(uint8_t* joint_id) {

}



void adisha::PoseEditor::torqueDisableAll() {

}



void adisha::PoseEditor::updateJointVal() {

}



void adisha::PoseEditor::savePose() {

}


void adisha::PoseEditor::exitApp() {
    rclcpp::shutdown();
    exit(0);
}



void adisha::PoseEditor::appTimerCallback() {
    /* App FSM */
    switch(this->app_state) {
        
        
        case adisha::PoseEditor::MAIN_MENU:
            std::cout << "=================[ Adisha Pose Editor ]=================" << std::endl;
            std::cout << "[1] See poses list" << std::endl;
            std::cout << "[2] Edit a pose" << std::endl;
            std::cout << "[3] Create a new pose" << std::endl;
            std::cout << "[4] Delete a pose" << std::endl;
            std::cout << "[5] Exit" << std::endl;
            std::cout << "input: ";
            std::cin >> this->char_buf;
            if(this->char_buf != '5') std::cout << ADISHA_POSE_EDITOR_CLEAR;
            break;

        
        case adisha::PoseEditor::EDIT_POSE:
            this->printPoseList();
            std::cout << "insert pose number to edit (c to cancel): ";
            std::cin >> this->char_buf;
            std::cout << ADISHA_POSE_EDITOR_CLEAR;
            break;


        case adisha::PoseEditor::CREATE_POSE:
            this->printPoseList();
            std::cout << "insert a new pose name            : ";
            std::cin >> this->str_buf;
            while(!this->createNewPose(this->str_buf.c_str())) {
                std::cout << "name is taken, create a new one   : ";
                std::cin >> this->str_buf;
            }
            std::cout << "insert a description for the pose : ";
            std::cin >> std::ws; std::getline(std::cin, this->str_buf);
            this->setPoseDesc(this->str_buf.c_str());
            std::cout << ADISHA_POSE_EDITOR_CLEAR;
            break;


        case adisha::PoseEditor::CONFIG_POSE:
            std::cout << "POSE NAME: " << this->pose_select.name << std::endl;
            std::cout << "POSE DESC: " << this->pose_select.desc << std::endl;
            std::cout << "==================================================" << std::endl;
            std::cout << "                Joint Values Table                " << std::endl;
            std::cout << "==================================================" << std::endl;
            std::cout << "ID\tJoint Name\tValue\tDur(ms)\tDelay(ms)" << std::endl;
            std::cout << "--------------------------------------------------" << std::endl;
            for(uint8_t i = 0; i < this->joint_num; ++i) {
                std::cout << (int)this->joint_id[i] << "\t" << this->joint_list[i] << "\t\t" << this->pose_select.motion[i][0] << "\t" << this->pose_select.motion[i][1] << "\t" << this->pose_select.motion[i][2] << std::endl;
            }
            std::cout << "--------------------------------------------------" << std::endl;
            std::cout << "COMMAND:" << std::endl;
            std::cout << "[off]             Disable all torques" << std::endl;
            std::cout << "[on]              Enable all torques" << std::endl;
            std::cout << "[off {Joint IDs}] Disable some torques" << std::endl;
            std::cout << "[on {Joint IDs}]  Enable some torques" << std::endl;
            std::cout << "[save]            Save" << std::endl;
            std::cout << "[exit]            Exit" << std::endl;
            std::cout << "input: ";
            std::cin >> std::ws; std::getline(std::cin, this->str_buf);
            this->parseCommand();
            std::cout << ADISHA_POSE_EDITOR_CLEAR;
            break;


        case adisha::PoseEditor::DELETE_POSE:
            this->printPoseList();
            std::cout << "insert pose number to delete (c to cancel): ";
            std::cin >> this->char_buf;
            break;
    }

    
    /* State changes */
    switch(this->app_state) {


        case adisha::PoseEditor::MAIN_MENU:
            if(this->char_buf == '1') {
                this->printPoseList();
            }

            else if(this->char_buf == '2') {
                this->app_state = adisha::PoseEditor::EDIT_POSE;
            }

            else if(this->char_buf == '3') {
                this->app_state = adisha::PoseEditor::CREATE_POSE;
            }

            else if(this->char_buf == '4') {
                this->app_state = adisha::PoseEditor::DELETE_POSE;
            }

            else if(this->char_buf == '5') {
                this->exitApp();
            }

            else {
                std::cout << "[ERROR] Invalid input" << std::endl << std::endl;
            }
            break;


        case adisha::PoseEditor::EDIT_POSE:
            if(this->char_buf == 'c' || this->char_buf == 'C') {
                this->app_state = adisha::PoseEditor::MAIN_MENU;
            }

            else if((uint64_t)(int(this->char_buf) - 49) < this->pose_list.size() && this->selectPose(this->pose_list[int(this->char_buf) - 49].c_str())) {
                this->app_state = adisha::PoseEditor::CONFIG_POSE;
            }

            else {
                std::cout << "[ERROR] Unable to select the desired pose" << std::endl << std::endl;
                this->app_state = adisha::PoseEditor::MAIN_MENU;
            }
            break;


        case adisha::PoseEditor::CREATE_POSE:
            this->app_state = adisha::PoseEditor::CONFIG_POSE;
            break;


        case adisha::PoseEditor::CONFIG_POSE:
            if(this->cmd_buf.size() == 1) {
                if(this->cmd_buf[0] == "exit") {
                    this->app_state = adisha::PoseEditor::MAIN_MENU;
                }
                else if(this->cmd_buf[0] == "on") {
                    this->torqueEnableAll();
                    std::cout << "[INFO] All torques enabled!" << std::endl << std::endl;
                    this->updateJointVal();
                }
                else if(this->cmd_buf[0] == "off") {
                    this->torqueDisableAll();
                    std::cout << "[INFO] All torques disabled!" << std::endl << std::endl;
                }
                else if(this->cmd_buf[0] == "save") {
                    this->savePose();
                    std::cout << "[INFO] Pose data saved successfully!" << std::endl << std::endl; 
                    this->app_state = adisha::PoseEditor::MAIN_MENU;
                }
                else {
                    std::cout << "[ERROR] Invalid command" << std::endl << std::endl;
                }
            }

            
            else if(this->cmd_buf.size() > 1) {
                if(this->cmd_buf[0] == "on") {
                    
                }

                else if(this->cmd_buf[1] == "off") {

                }

                else {

                }
            }


            else {

            }
            break;


        case adisha::PoseEditor::DELETE_POSE:
            if(this->char_buf == 'c' || this->char_buf == 'C') {
                this->app_state = adisha::PoseEditor::MAIN_MENU;
            }
            break;
    }


    /* Reset input */
    this->char_buf = '_';
    this->str_buf.clear();
    this->cmd_buf.clear();
}









/*------------------------------------------
Private Methods
--------------------------------------------*/
void adisha::PoseEditor::printPoseList() {
    std::cout << "=================================" << std::endl;
    std::cout << "            POSE LIST            " << std::endl;
    std::cout << "=================================" << std::endl;
    std::cout << "  No.\tPose Name" << std::endl;
    std::cout << "---------------------------------" << std::endl;
    uint16_t no = 1;
    for(const auto& pose : this->pose_list) {
        std::cout << "  " << no << ".\t" + (std::string)pose << std::endl;
        ++no;
    }
    std::cout << "---------------------------------" << std::endl << std::endl;
}



void adisha::PoseEditor::parseCommand() {
    std::istringstream iss(this->str_buf);
    std::string word;
    while(iss >> word) {
        this->cmd_buf.push_back(word);
    }
}