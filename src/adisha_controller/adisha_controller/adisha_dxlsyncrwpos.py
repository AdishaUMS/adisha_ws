from .submodule.adisha_dxlsyncrwpos_node import *



def main(args=None):
    # Initiate rclpy
    rclpy.init(args=args)

    # Create the node
    adisha_dxlsyncrwpos_node = AdishaDxlSyncRWPos()

    # Spin the node
    rclpy.spin(adisha_dxlsyncrwpos_node)

    # Shutdown and close everything
    adisha_dxlsyncrwpos_node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()