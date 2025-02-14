# CMPUT412

## About

This is the repo where I'm going to keep track my progress for CMPUT 412 winter 2025. Each subfolder in the repo should represent an exercise. Here is my website for this course. It contains reports about all exercises.

```bash
https://sites.google.com/ualberta.ca/cmput412-hanransong/home
```

## Steps to Set Up and Connect

1. **Source dts**  
   Run the following commands in your terminal to configure the environment:

   ```bash
   export PATH=~/.local/bin:${PATH}
   source ~/.bashrc

2. **Connect to Dockiebot Dashboard**  
   In browser enter:

   ```bash
   http://csc22919.local/

3. **Connect through ssh**  
   in terminal enter:

   ```bash
   ssh duckie@csc22919.local
   ```

   The default password is:

   ```bash
   quackquack

## Common Commands

1. **Make it move**

   ```bash
   dts duckiebot keyboard_control csc22919
   ```

2. **Make it see**

   ```bash
   dts start_gui_tools csc22919
   rqt_image_view
   ```

2. **Compile project**

   ```bash
   dts devel build -f
   ```

3. **Launch the node**

   ```bash
   dts devel run -R csc22919 -L NODE_NAME
   ```

4. **dts-gui-tools failed to recognize robot**   
   In dt_gui_tools   

   ```bash
   export ROS_MASTER_URI=http://csc22919.local:11311/
   ```

## Visulization

1. **Record rosbag**   
    Remember to delete previous bag. It won't overwrite the old bag.     
   In dt_gui_tools
   
   ```bash
   rosbag record -O BAG_NAME.bag /csc22919/wheels_driver_node/wheels_cmd /csc22919/left_wheel_encoder_node/tick /csc22919/right_wheel_encoder_node/tick
   ```

3. **Download rosbag**   
   Outside dt_gui_tools

   ```bash
   docker cp CONTAINER_ID:/code/catkin_ws/src/dt-gui-tools/BAG_NAME.bag /home/hanran/Documents/UA/Winter2025/SAVE_LOCATION/
   ```

4. **Visulize**  
   Run the visulization script. Make sure you pass in the correct bag name and set plot title.

## Turn Off or Reboot

1. **Turn off**
   ```bash
   dts duckiebot shutdown csc22919

2. **Reboot**
   ```bash
   dts duckiebot reboot csc22919

## Some Useful Websites

1. **The Duckiebot Operation Manual**

   ```bash
   https://docs.duckietown.com/daffy/opmanual-duckiebot/intro.html

2. **Hands-on Robotics Development using Duckietown**

   ```bash
   https://docs-old.duckietown.org/daffy/duckietown-robotics-development/out/index.html
