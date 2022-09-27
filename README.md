# Carla collector

Lightweight system for gathering data using CARLA. Should allow you to effortlessly gather as much data as you want. 

To use, make a new config file. The keys should be pretty self-explanatory if you look at the samples. Use a config with

`python main.py configs/my_config.json`

This will automatically start an instance of CARLA, and connect to it before gathering data. You should see something like this:

![image](https://user-images.githubusercontent.com/88854626/192413713-81704341-a060-4476-affe-c11a89b4baca.png)

To stop gathering data, terminate the program with CNTRL+C. This should kill the instance of CARLA, and then kill the program.


