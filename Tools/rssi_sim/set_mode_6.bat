@echo on
cd "%USERPROFILE%\Documents\GitHub\ardupilot\Tools\rssi_sim"
python ".\set_mode_com.py --port COM7 --baud 115200 --mode 6" 
