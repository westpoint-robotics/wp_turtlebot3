# WP Raspberry Pi 5 maximum current fix

# Background

- The Raspberry Pi5 to operate at its fullest capability requires a power supply that provides upto 5A at 5V of power.
- To solve this on the WP Turtlebot we added a buck converter that can provide upto 15A at 5V.
- The Raspberry Pi5 may limit its performance if does not know that 5V 5A is available.

# Solution

- Check the eeprom settings to see if the Raspberry Pi5 assumes it can draw 5A.  
  `od -A n -d -j 2 --endian=big /proc/device-tree/chosen/power/max_current`
- If this returns `5000` all is good and no further action is required.
- If this returns `3000` then run the below command to open the eeprom editor  
  `sudo rpi-eeprom-config --edit`
- Add the below line in the editor:  
  `PSU_MAX_CURRENT=5000`  
- Exit and save by pressing CTRL-x, and enter `y` to save.
- Reboot the Raspberry Pi 5
- Check that the eeprom settings persisted by looking for the value `5000` upon entering the below command:  
  `od -A n -d -j 2 --endian=big /proc/device-tree/chosen/power/max_current`
