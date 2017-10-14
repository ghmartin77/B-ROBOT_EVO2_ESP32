# B-ROBOT_EVO2_ESP32
ESP32 port of JJRobots brilliant B-ROBOT_EVO2

The robot:
![Robot](assets/Robot.png)

Circuit top:
![Circuit Top](assets/CircuitTop.png)

Circuit bottom:
![Circuit Bottom](assets/CircuitBottom.png)


Pin connections (refer to defines.h to change):
* Enable motors: P12
* Servo: P17
* Motor1 Dir: P27
* Motor1 Step: P14
* Motor2 Dir: P25
* Motor2 Step: P26
* I2C pins for gyro: (defaults) SDA=P21, SCL=P22

ESP-32 Board is a NodeMCU ESP32s, see [Pinout](http://esp32.net/images/Ai-Thinker/NodeMCU-32S/Ai-Thinker_NodeMCU-32S_DiagramPinout.png).
