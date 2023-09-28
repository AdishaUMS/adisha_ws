import yaml


ADISHA_ROBOT_INFO_YAML_PATH = 'src/adisha_main/config/adisha_robot_info.yaml'
ADISHA_JOINT_INFO_YAML_PATH = 'src/adisha_controller/config/adisha_joints_info.yaml'
ADISHA_POSE_YAML_PATH       = 'src/adisha_controller/config/adisha_pose.yaml'
ADISHA_MOTION_SEQUENCE_PATH = 'src/adisha_controller/config/motion_sequence/'


ADISHA_ROBOT_INFO_YAML      = yaml.safe_load(open(ADISHA_ROBOT_INFO_YAML_PATH, 'r'))
ADISHA_JOINT_INFO_YAML      = yaml.safe_load(open(ADISHA_JOINT_INFO_YAML_PATH, 'r'))
ADISHA_POSE_YAML            = yaml.safe_load(open(ADISHA_POSE_YAML_PATH, 'r'))


ADISHA_ROBOT_NAME           = ADISHA_ROBOT_INFO_YAML['name']
ADISHA_JOINT_NUM            = ADISHA_JOINT_INFO_YAML['joint_num']
ADISHA_JOINT_LIST           = ADISHA_JOINT_INFO_YAML['joint_list']
ADISHA_JOINT_ID             = ADISHA_JOINT_INFO_YAML['joint_id']
ADISHA_JOINT_SERVO          = []
ADISHA_DXL_ERROR            = 99999

ADISHA_XL320    = 0
ADISHA_AX12A    = 1
ADISHA_MX28     = 2
for servo_name in ADISHA_JOINT_INFO_YAML['joint_servo']:
    if servo_name == 'XL320':
        ADISHA_JOINT_SERVO.append(ADISHA_XL320)
    elif servo_name == 'AX12A':
        ADISHA_JOINT_SERVO.append(ADISHA_AX12A)
    elif servo_name == 'MX28':
        ADISHA_JOINT_SERVO.append(ADISHA_MX28)


XL320_ADDR_TORQUE_ENABLE    = ADISHA_JOINT_INFO_YAML['XL320']['addr_torque_enable']
AX12A_ADDR_TORQUE_ENABLE    = ADISHA_JOINT_INFO_YAML['AX12A']['addr_torque_enable']
MX28_ADDR_TORQUE_ENABLE     = ADISHA_JOINT_INFO_YAML['MX28']['addr_torque_enable']

XL320_SIZE_TORQUE_ENABLE    = ADISHA_JOINT_INFO_YAML['XL320']['size_torque_enable']
AX12A_SIZE_TORQUE_ENABLE    = ADISHA_JOINT_INFO_YAML['AX12A']['size_torque_enable']
MX28_SIZE_TORQUE_ENABLE     = ADISHA_JOINT_INFO_YAML['MX28']['size_torque_enable']

XL320_ADDR_GOAL_POS         = ADISHA_JOINT_INFO_YAML['XL320']['addr_goal_pos']
AX12A_ADDR_GOAL_POS         = ADISHA_JOINT_INFO_YAML['AX12A']['addr_goal_pos']
MX28_ADDR_GOAL_POS          = ADISHA_JOINT_INFO_YAML['MX28']['addr_goal_pos']

XL320_SIZE_GOAL_POS         = ADISHA_JOINT_INFO_YAML['XL320']['size_goal_pos']
AX12A_SIZE_GOAL_POS         = ADISHA_JOINT_INFO_YAML['AX12A']['size_goal_pos']
MX28_SIZE_GOAL_POS          = ADISHA_JOINT_INFO_YAML['MX28']['size_goal_pos']

XL320_ADDR_PRESENT_POS      = ADISHA_JOINT_INFO_YAML['XL320']['addr_present_pos']
AX12A_ADDR_PRESENT_POS      = ADISHA_JOINT_INFO_YAML['AX12A']['addr_present_pos']
MX28_ADDR_PRESENT_POS       = ADISHA_JOINT_INFO_YAML['MX28']['addr_present_pos']

XL320_SIZE_PRESENT_POS      = ADISHA_JOINT_INFO_YAML['XL320']['size_present_pos']
AX12A_SIZE_PRESENT_POS      = ADISHA_JOINT_INFO_YAML['AX12A']['size_present_pos']
MX28_SIZE_PRESENT_POS       = ADISHA_JOINT_INFO_YAML['MX28']['size_present_pos']