# Devito
Group project with myself, Shivam Bhatnagar, Renke Huang, Ian Kegler, Benjamin Pheifer, & Tessa Smulders.  
This repositoty contains python code used to control the Baxter Research Robot using the HTC VIVE only. Additional PCs and software packages e.g. ROS are used.    

Project Overview: https://youtu.be/8fvgJwRspsM  
Operation guide: https://youtu.be/LIy24xF3gOI  


#### cal_frame_broadcaster.py
Run this file first.  
Continuously broadcasts newly calibrated base frame to ROS /tf.  

#### DEVITO_main.py  
DEVITO main file, ran after cal_frame_broadcaster.py, in conjunction with VIVE_grip_cont.py and head_pan.py.  
Provides primary functionality in teleoperating DENIRO using HTC VIVE.  

#### head_pan.py
Node to control the movement of DENIRO's screen about the Z axis based on VIVE headset orientation.  

#### move_joints_to.py
Movement Test file. If DE NIRO does not move, turn it off and on again.

#### VIVE_grip_cont.py
Node to control the movement of DENIRO's grippers based on the VIVE controller's Triggers.  
