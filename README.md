# HC-SR04 UltraSonic Sensor - ROS in Python
+ [HC-SR04 specification and guide](https://randomnerdtutorials.com/complete-guide-for-ultrasonic-sensor-hc-sr04/)
+ [Circuit and Python original Source code reference](https://blog.naver.com/roboholic84/220319850312)
</br></br><br>

## ● Circuit
+ Raspberry pi cannot get input for 5V but 3.3V so we have to reduce voltage using Resistance like [here](https://blog.naver.com/roboholic84/220319850312)
<p align="center">
<img src="https://github.com/engcang/image-files/blob/master/sonar_sensor/Resistance.jpg" width="500" hspace="0"/>
</p>
<br>
+ Plug into GPIO, for my code, 
<p align="center">
<img src="https://github.com/engcang/image-files/blob/master/sonar_sensor/gpio.png" width="500" hspace="0"/>
</p>
  + Vcc to any 5V (pin number 2)
  + GND to any Ground (pin number 9)
  + TRIG to GPIO 27 (pin number 13)
  + Echo to GPIO 17 (pin number 11)


<p align="center">
<img src="https://github.com/engcang/image-files/blob/master/sonar_sensor/Raspberry.jpg" width="500" hspace="0"/>
</p>

</br></br>


## ● Using the code as ROS node
+ git clone the codes first
  ~~~shell
  $ git clone https://github.com/engcang/HC-SR04-UltraSonicSensor-ROS-RaspberryPi.git
  ~~~
<br>

+ Run the code directly with ROS Master
  ~~~shell
  $ roscore
  $ python ROS_sonar_sensor.py
  ~~~
<br>

+ Run the code after make it as Node
  ~~~shell
  $ cd ~/catkin_ws/src
  $ catkin_create_pkg <name> rospy roslib std_msgs
  $ cd ~/catkin_ws && catkin_make
  $ cd ~/catkin_ws/src/<name> && mkdir scripts
  $ mv ~/HC-SR04-UltraSonicSensor-ROS-RaspberryPi/ROS_sonar_sensor.py ~/catkin_ws/src/<name>/scripts
  $ chmod +x ROS_sonar_sensor.py
  $ roscore
  $ rosrun <name> ROS_sonar_sensor.py
  ~~~
<br>

+ Run the code by ROS launch
  ~~~xml
  <node pkg="<name>" type="ROS_sonar_sensor.py" name="ROS_sonar_sensor" />
  ~~~
<p align="center">
<img src="https://github.com/engcang/image-files/blob/master/sonar_sensor/ROS_topic.gif" width="400" height="500" hspace="0"/>
</p>
