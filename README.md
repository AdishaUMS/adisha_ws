# adhisa_ws
This repository is a ROS2 workspace for the Adisha UMS robot's development. This workspace was built and tested with ROS2 Humble and Ubuntu 22.04.


## I. Installation Prerequisites
The following installations need to be done for running the workspace:

*1. Dynamixel SDK*
    
    You can either go to this [download link](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/download/) or just do the following commands:
    ```
    git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
    cd DynamixelSDK/python/         # or cd DynamixelSDK-master/python/
    sudo python3 setup.py install   # or sudo python setup.py install 
    ```

*2. node.js*

    > [!NOTE]
    > It is recommended to use nvm (Node Version Manager) to install node.js instead of apt.
    > 
    > Install nvm from this [Github repository](https://github.com/nvm-sh/nvm)
    
    Install the latest long term support of node.js with:
    ```
    nvm install --lts=current
    ```
    Check node and npm version with:
    ```
    node -v
    npm -v
    ```

## II. Launches