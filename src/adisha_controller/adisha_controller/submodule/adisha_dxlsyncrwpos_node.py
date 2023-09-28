import rclpy
from rclpy.node import Node
from .adisha_controller_config import *
import adisha_interface.msg as adisha_msgs
import dynamixel_sdk as dxlsdk



class AdishaDxlSyncRWPos(Node):



    def __init__(self) -> None:
        super().__init__(f'{ADISHA_ROBOT_NAME}_DXLSyncRWPos')

        
        # Constants
        self.USB_DEVICE     = '/dev/ttyUSB0'
        self.DXL_PROTOCOL_1 = 1.0
        self.DXL_PROTOCOL_2 = 2.0
        self.DXL_BAUDRATE   = 1000000
        self.TIMER1_RATE    = 1./30.

        
        # Create ROS2 publishers and subscribers
        self.present_pos_pub    = self.create_publisher(adisha_msgs.JointVal, f'/{ADISHA_ROBOT_NAME}/joint/present_pos', 10)
        self.torque_enable_sub  = self.create_subscription(adisha_msgs.JointTorque, f'/{ADISHA_ROBOT_NAME}/joint/torque_enable', self.cbJointTorqueEnable, 10)
        self.goal_pos_sub       = self.create_subscription(adisha_msgs.JointVal, f'/{ADISHA_ROBOT_NAME}/joint/goal_pos', self.cbJointGoalPos, 10)
        

        # Create ROS2 timers
        # timer1 used for publishing present position
        self.timer1_routine     = self.create_timer(self.TIMER1_RATE, self.timer1Routine)


        # Messages
        self.present_pos_msg    = adisha_msgs.JointVal()


        # Dynamixel SDK interfaces
        self.dxl_packet_handler1    = dxlsdk.PacketHandler(self.DXL_PROTOCOL_1)
        self.dxl_packet_handler2    = dxlsdk.PacketHandler(self.DXL_PROTOCOL_2)
        self.dxl_port_handler       = dxlsdk.PortHandler(self.USB_DEVICE)
        
        if self.dxl_port_handler.openPort():
            self.get_logger().info(f'Succeeded to open {self.USB_DEVICE} port!')
        else:
            self.get_logger().error(f'Failed to open {self.USB_DEVICE} port!')
            self.get_logger().error('Node destroyed.')
            self.destroy_node()
            exit()

        if self.dxl_port_handler.setBaudRate(self.DXL_BAUDRATE):
            self.get_logger().info(f'Succeeded to change the baudrate to {self.DXL_BAUDRATE}!')
        else:
            self.get_logger().error(f'Failed to change the baudrate to {self.DXL_BAUDRATE}!')
            self.get_logger().error('Node destroyed.')
            self.destroy_node()
            exit()

        # Note: in Dynamixel protocol 1.0, both AX-12A and MX-28 have the same address and data size
        self.gsw2_arm_torque_enable     = dxlsdk.GroupSyncWrite(self.dxl_port_handler, self.dxl_packet_handler2, XL320_ADDR_TORQUE_ENABLE, XL320_SIZE_TORQUE_ENABLE)
        self.gsw1_leg_torque_enable     = dxlsdk.GroupSyncWrite(self.dxl_port_handler, self.dxl_packet_handler1, MX28_ADDR_TORQUE_ENABLE, MX28_SIZE_TORQUE_ENABLE)

        self.gsw2_arm_goal_pos          = dxlsdk.GroupSyncWrite(self.dxl_port_handler, self.dxl_packet_handler2, XL320_ADDR_GOAL_POS, XL320_SIZE_GOAL_POS)
        self.gsw1_leg_goal_pos          = dxlsdk.GroupSyncWrite(self.dxl_port_handler, self.dxl_packet_handler1, MX28_ADDR_GOAL_POS, MX28_SIZE_GOAL_POS)

        self.gsr2_arm_present_pos       = dxlsdk.GroupSyncRead(self.dxl_port_handler, self.dxl_packet_handler2, XL320_ADDR_PRESENT_POS, XL320_SIZE_PRESENT_POS)
        self.gbr1_leg_present_pos       = dxlsdk.GroupBulkRead(self.dxl_port_handler, self.dxl_packet_handler1)

        # Loop through all group sync read and group bulk read to assign ID's
        for idx in range(ADISHA_JOINT_NUM):

            if ADISHA_JOINT_SERVO[idx] == ADISHA_XL320:
                self.gsr2_arm_present_pos.addParam(ADISHA_JOINT_ID[idx])

            elif ADISHA_JOINT_SERVO[idx] == ADISHA_AX12A or ADISHA_JOINT_SERVO[idx] == ADISHA_MX28:
                self.gbr1_leg_present_pos.addParam(ADISHA_JOINT_ID[idx], MX28_ADDR_PRESENT_POS, MX28_SIZE_PRESENT_POS)


    
    def setTorqueEnable(self, torque_values:list) -> None:
        for idx in range(ADISHA_JOINT_NUM):
            # Torque enable is a 1-byte parameter
            param = [dxlsdk.DXL_LOBYTE(torque_values[idx])]

            if ADISHA_JOINT_SERVO[idx] == ADISHA_XL320:
                dxl_result = self.gsw2_arm_torque_enable.addParam(ADISHA_JOINT_ID[idx], param)
                if not dxl_result:
                    self.get_logger().error(f'Torque enable addparam failed [ID:{ADISHA_JOINT_ID[idx]}, NAME:{ADISHA_JOINT_LIST[idx]}]')

            elif ADISHA_JOINT_SERVO[idx] == ADISHA_AX12A or ADISHA_JOINT_SERVO[idx] == ADISHA_MX28:
                dxl_result = self.gsw1_leg_torque_enable.addParam(ADISHA_JOINT_ID[idx], param)
                if not dxl_result:
                    self.get_logger().error(f'Torque enable addparam failed [ID:{ADISHA_JOINT_ID[idx]}, NAME:{ADISHA_JOINT_LIST[idx]}]')
        

        # Send packet
        dxl_result = self.gsw2_arm_torque_enable.txPacket()
        if dxl_result != dxlsdk.COMM_SUCCESS:
            self.get_logger().error('Arm torque enable txPacket failed!')
            self.get_logger().error(f'{self.dxl_packet_handler2.getTxRxResult(dxl_result)}')

        dxl_result = self.gsw1_leg_torque_enable.txPacket()
        if dxl_result != dxlsdk.COMM_SUCCESS:
            self.get_logger().error('Leg torque enable txPacket failed!')
            self.get_logger().error(f'{self.dxl_packet_handler1.getTxRxResult(dxl_result)}')


        # Clear group sync write buffer
        self.gsw2_arm_torque_enable.clearParam()
        self.gsw1_leg_torque_enable.clearParam()



    def setGoalPos(self, goal_pos:list) -> None:
        for idx in range(ADISHA_JOINT_NUM):
            # All of the servo's in our particular case have the same size of goal position, 2-bytes
            param = [dxlsdk.DXL_LOBYTE(goal_pos[idx]), dxlsdk.DXL_HIBYTE(goal_pos[idx])]

            if ADISHA_JOINT_SERVO[idx] == ADISHA_XL320:
                dxl_result = self.gsw2_arm_goal_pos.addParam(ADISHA_JOINT_ID[idx], param)
                if not dxl_result:
                    self.get_logger().error(f'Goal pos addparam failed [ID:{ADISHA_JOINT_ID[idx]}, NAME:{ADISHA_JOINT_LIST[idx]}]')

            elif ADISHA_JOINT_SERVO[idx] == ADISHA_AX12A or ADISHA_JOINT_SERVO[idx] == ADISHA_MX28:
                dxl_result = self.gsw1_leg_goal_pos.addParam(ADISHA_JOINT_ID[idx], param)
                if not dxl_result:
                    self.get_logger().error(f'Goal pos addparam failed [ID:{ADISHA_JOINT_ID[idx]}, NAME:{ADISHA_JOINT_LIST[idx]}]')


        # Send packet
        dxl_result = self.gsw2_arm_goal_pos.txPacket()
        if dxl_result != dxlsdk.COMM_SUCCESS:
            self.get_logger().error('Arm goal pos txPacket failed!')
            self.get_logger().error(f'{self.dxl_packet_handler2.getTxRxResult(dxl_result)}')

        dxl_result = self.gsw1_leg_goal_pos.txPacket()
        if dxl_result != dxlsdk.COMM_SUCCESS:
            self.get_logger().error('Leg goal pos txPacket failed!')
            self.get_logger().error(f'{self.dxl_packet_handler1.getTxRxResult(dxl_result)}')


        # Clear group sync write buffer
        self.gsw2_arm_goal_pos.clearParam()
        self.gsw1_leg_goal_pos.clearParam()



    def getPresentPos(self) -> list:
        # Get packets
        dxl_result = self.gsr2_arm_present_pos.txRxPacket()
        if dxl_result != dxlsdk.COMM_SUCCESS:
            self.get_logger().error('Arm present pos txRxPacket failed!')
            self.get_logger().error(f'{self.dxl_packet_handler2.getTxRxResult(dxl_result)}')

        dxl_result = self.gbr1_leg_present_pos.txRxPacket()
        if dxl_result != dxlsdk.COMM_SUCCESS:
            self.get_logger().error('Arm present pos txRxPacket failed!')
            self.get_logger().error(f'{self.dxl_packet_handler1.getTxRxResult(dxl_result)}')


        # Loop through all joints
        present_pos_data = []
        for idx in range(ADISHA_JOINT_NUM):
            if ADISHA_JOINT_SERVO[idx] == ADISHA_XL320:
                dxl_result = self.gsr2_arm_present_pos.isAvailable(ADISHA_JOINT_ID[idx], XL320_ADDR_PRESENT_POS, XL320_SIZE_PRESENT_POS)
                if not dxl_result:
                    self.get_logger().error(f'Present pos getData failed [ID:{ADISHA_JOINT_ID[idx]}, NAME:{ADISHA_JOINT_LIST[idx]}]')
                    present_pos_data.append(ADISHA_DXL_ERROR)
                else:
                    present_pos_data.append(self.gsr2_arm_present_pos.getData(ADISHA_JOINT_ID[idx], XL320_ADDR_PRESENT_POS, XL320_SIZE_PRESENT_POS))

            elif ADISHA_JOINT_SERVO[idx] == ADISHA_AX12A or ADISHA_JOINT_SERVO[idx] == ADISHA_MX28:
                dxl_result = self.gbr1_leg_present_pos.isAvailable(ADISHA_JOINT_ID[idx], MX28_ADDR_PRESENT_POS, MX28_SIZE_PRESENT_POS)
                if not dxl_result:
                    self.get_logger().error(f'Present pos getData failed [ID:{ADISHA_JOINT_ID[idx]}, NAME:{ADISHA_JOINT_LIST[idx]}]')
                    present_pos_data.append(ADISHA_DXL_ERROR)
                else:
                    present_pos_data.append(self.gbr1_leg_present_pos.getData(ADISHA_JOINT_ID[idx], MX28_ADDR_PRESENT_POS, MX28_SIZE_PRESENT_POS))
        

        # Return the data for publishing
        return present_pos_data



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
        self.setTorqueEnable(torque_values=torque_values)



    def cbJointGoalPos(self, msg) -> None:
        goal_pos = [
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
        self.setGoalPos(goal_pos=goal_pos)



    def timer1Routine(self) -> None:
        # Assign values
        present_post_data = self.getPresentPos()
        self.present_pos_msg.head       = present_post_data[0]
        self.present_pos_msg.neck       = present_post_data[1]
        self.present_pos_msg.l_sho_r    = present_post_data[2]
        self.present_pos_msg.r_sho_r    = present_post_data[3]
        self.present_pos_msg.l_sho_p    = present_post_data[4]
        self.present_pos_msg.r_sho_p    = present_post_data[5]
        self.present_pos_msg.l_arm_y    = present_post_data[6]
        self.present_pos_msg.r_arm_y    = present_post_data[7]
        self.present_pos_msg.l_arm_p    = present_post_data[8]
        self.present_pos_msg.r_arm_p    = present_post_data[9]
        self.present_pos_msg.l_wrist    = present_post_data[10]
        self.present_pos_msg.r_wrist    = present_post_data[11]
        self.present_pos_msg.l_hip_y    = present_post_data[12]
        self.present_pos_msg.r_hip_y    = present_post_data[13]
        self.present_pos_msg.l_hip_r    = present_post_data[14]
        self.present_pos_msg.r_hip_r    = present_post_data[15]
        self.present_pos_msg.l_hip_p    = present_post_data[16]
        self.present_pos_msg.r_hip_p    = present_post_data[17]
        self.present_pos_msg.l_knee     = present_post_data[18]
        self.present_pos_msg.r_knee     = present_post_data[19]
        self.present_pos_msg.l_ank_p    = present_post_data[20]
        self.present_pos_msg.r_ank_p    = present_post_data[21]
        self.present_pos_msg.l_ank_r    = present_post_data[22]
        self.present_pos_msg.r_ank_r    = present_post_data[23]

        # Publish
        self.present_pos_pub.publish(self.present_pos_msg)