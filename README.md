# CMPUT412

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
