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
