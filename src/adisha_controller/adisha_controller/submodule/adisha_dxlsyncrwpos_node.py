import rclpy
from rclpy.node import Node
from .adisha_controller_config import *
import adisha_interface.msg as adisha_msg
import dynamixel_sdk as dxlsdk



class AdishaDxlSyncRWPos(Node):



    def __init__(self) -> None:
        super().__init__(f'{ROBOT_NAME}_DXLSyncRWPos')

        
        # Constants
        self.SPIN_RATE      = 1./30.
        self.USB_DEVICE     = '/dev/ttyUSB0'
        self.DXL_PROTOCOL_1 = 1.0
        self.DXL_PROTOCOL_2 = 2.0
        self.DXL_BAUDRATE   = 1000000

        
        # Create ROS2 instances
        self.present_pos_pub    = self.create_publisher(adisha_msg.JointVal, f'/{ROBOT_NAME}/joint/present_pos', 10)
        self.torque_enable_sub  = self.create_subscription(adisha_msg.JointTorque, f'/{ROBOT_NAME}/joint/torque_enable', self.cbJointTorqueEnable, 10)
        self.goal_pos_sub       = self.create_subscription(adisha_msg.JointVal, f'/{ROBOT_NAME}/joint/goal_pos', self.cbJointGoalPos, 10)
        self.timer_routine      = self.create_timer(self.SPIN_RATE, self.timerRoutine)

        # self.test_pub           = self.create_publisher(adisha_msg.JointTorque, f'/{ROBOT_NAME}/joint/torque_enable', 10)
        # self.test_msg           = adisha_msg.JointTorque()
        # self.test_msg.head      = True
        # self.test_msg.neck      = True
        # self.test_msg.l_sho_r   = True
        # self.test_msg.r_sho_r   = True
        # self.test_msg.l_sho_p   = True
        # self.test_msg.r_sho_p   = True
        # self.test_msg.l_arm_y   = True
        # self.test_msg.r_arm_y   = True
        # self.test_msg.l_arm_p   = True
        # self.test_msg.r_arm_p   = True
        # self.test_msg.l_wrist   = True
        # self.test_msg.r_wrist   = True
        # self.test_msg.l_hip_y   = True
        # self.test_msg.r_hip_y   = True 
        # self.test_msg.l_hip_r   = True  
        # self.test_msg.r_hip_r   = True  
        # self.test_msg.l_hip_p   = True  
        # self.test_msg.r_hip_p   = True  
        # self.test_msg.l_knee    = True
        # self.test_msg.r_knee    = True
        # self.test_msg.l_ank_p   = True
        # self.test_msg.r_ank_p   = True
        # self.test_msg.l_ank_r   = True
        # self.test_msg.r_ank_r   = True


        # Dynamixel SDK interfaces
        self.dxl_port_handler       = dxlsdk.PortHandler(self.USB_DEVICE)   
        self.dxl_packet_handler1    = dxlsdk.PacketHandler(self.DXL_PROTOCOL_1)
        self.dxl_packet_handler2    = dxlsdk.PacketHandler(self.DXL_PROTOCOL_2)
        self.dxl_port_handler.openPort()
        self.dxl_port_handler.setBaudRate(self.DXL_BAUDRATE)


        self.gsw2_arm_torque_enable     = dxlsdk.GroupSyncWrite(self.dxl_port_handler, self.dxl_packet_handler2, XL320_ADDR_TORQUE_ENABLE, XL320_SIZE_TORQUE_ENABLE)
        self.gsw1_uleg_torque_enable    = dxlsdk.GroupSyncWrite(self.dxl_port_handler, self.dxl_packet_handler1, AX12A_ADDR_TORQUE_ENABLE, AX12A_SIZE_TORQUE_ENABLE)
        self.gsw1_lleg_torque_enable    = dxlsdk.GroupSyncWrite(self.dxl_port_handler, self.dxl_packet_handler1, MX28_ADDR_TORQUE_ENABLE, MX28_SIZE_TORQUE_ENABLE)

        self.gsw2_arm_goal_pos          = dxlsdk.GroupSyncWrite(self.dxl_port_handler, self.dxl_packet_handler2, XL320_ADDR_GOAL_POS, XL320_SIZE_GOAL_POS)
        self.gsw1_uleg_goal_pos         = dxlsdk.GroupSyncWrite(self.dxl_port_handler, self.dxl_packet_handler1, AX12A_ADDR_GOAL_POS, AX12A_SIZE_GOAL_POS)
        self.gsw1_lleg_goal_pos         = dxlsdk.GroupSyncWrite(self.dxl_port_handler, self.dxl_packet_handler1, MX28_ADDR_GOAL_POS, MX28_SIZE_GOAL_POS)

        self.gsr2_arm_present_pos       = dxlsdk.GroupSyncRead(self.dxl_port_handler, self.dxl_packet_handler2, XL320_ADDR_PRESENT_POS, XL320_SIZE_PRESENT_POS)
        self.gbr1_leg_present_pos       = dxlsdk.GroupBulkRead(self.dxl_port_handler, self.dxl_packet_handler1)


    
    def setTorqueEnable(self, torque_values:list) -> None:
        for idx in range(JOINT_NUM):
            param = [dxlsdk.DXL_LOBYTE(dxlsdk.DXL_LOWORD(torque_values[idx]))]
            if JOINT_SERVO[idx] == 'XL320':
                self.gsw2_arm_torque_enable.addParam(JOINT_ID[idx], param)

            elif JOINT_SERVO[idx] == 'AX12A':
                self.gsw1_uleg_torque_enable.addParam(JOINT_ID[idx], param)

            if JOINT_SERVO[idx] == 'MX28':    
                self.gsw1_lleg_torque_enable.addParam(JOINT_ID[idx], param)
        
        # Send packet
        self.gsw2_arm_torque_enable.txPacket()
        self.gsw1_uleg_torque_enable.txPacket()
        self.gsw1_lleg_torque_enable.txPacket()

        # Clear group sync write buffer
        self.gsw2_arm_torque_enable.clearParam()
        self.gsw1_uleg_torque_enable.clearParam()
        self.gsw1_lleg_torque_enable.clearParam()



    def cbJointTorqueEnable(self, msg) -> None:
        torque_values = [
            msg.head,
            msg.neck,
            msg.l_sho_r,
            msg.r_sho_r,
            msg.l_sho_p,
            msg.r_sho_p,
            msg.l_arm_y,
            msg.r_arm_y,
            msg.l_arm_p,
            msg.r_arm_p,
            msg.l_wrist,
            msg.r_wrist,
            msg.l_hip_y,
            msg.r_hip_y,
            msg.l_hip_r,
            msg.r_hip_r,
            msg.l_hip_p,
            msg.r_hip_p,
            msg.l_knee,
            msg.r_knee,
            msg.l_ank_p,
            msg.r_ank_p,
            msg.l_ank_r,
            msg.r_ank_r
        ]
        self.setTorqueEnable(torque_values)



    def cbJointGoalPos(self, msg) -> None:
        pass



    def timerRoutine(self) -> None:
        pass
        # self.test_pub.publish(self.test_msg)
        # self.test_msg.head      = not self.test_msg.head
        # self.test_msg.neck      = not self.test_msg.neck   
        # self.test_msg.l_sho_r   = not self.test_msg.l_sho_r
        # self.test_msg.r_sho_r   = not self.test_msg.r_sho_r
        # self.test_msg.l_sho_p   = not self.test_msg.l_sho_p
        # self.test_msg.r_sho_p   = not self.test_msg.r_sho_p
        # self.test_msg.l_arm_y   = not self.test_msg.l_arm_y
        # self.test_msg.r_arm_y   = not self.test_msg.r_arm_y
        # self.test_msg.l_arm_p   = not self.test_msg.l_arm_p
        # self.test_msg.r_arm_p   = not self.test_msg.r_arm_p
        # self.test_msg.l_wrist   = not self.test_msg.l_wrist
        # self.test_msg.r_wrist   = not self.test_msg.r_wrist
        # self.test_msg.l_hip_y   = not self.test_msg.l_hip_y
        # self.test_msg.r_hip_y   = not self.test_msg.r_hip_y 
        # self.test_msg.l_hip_r   = not self.test_msg.l_hip_r  
        # self.test_msg.r_hip_r   = not self.test_msg.r_hip_r  
        # self.test_msg.l_hip_p   = not self.test_msg.l_hip_p  
        # self.test_msg.r_hip_p   = not self.test_msg.r_hip_p 
        # self.test_msg.l_knee    = not self.test_msg.l_knee 
        # self.test_msg.r_knee    = not self.test_msg.r_knee
        # self.test_msg.l_ank_p   = not self.test_msg.l_ank_p
        # self.test_msg.r_ank_p   = not self.test_msg.r_ank_p
        # self.test_msg.l_ank_r   = not self.test_msg.l_ank_r 
        # self.test_msg.r_ank_r   = not self.test_msg.r_ank_r