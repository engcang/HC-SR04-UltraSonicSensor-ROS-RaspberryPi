# HC-SR04 UltraSonic Sensor - ROS in Python
+ [HC-SR04 specification and guide](https://randomnerdtutorials.com/complete-guide-for-ultrasonic-sensor-hc-sr04/)
+ [Circuit and Python original Source code reference](https://blog.naver.com/roboholic84/220319850312)
</br></br><br>

## Index
+ [Circuit](#-circuit)
+ [Code explanation](#-code-explanation)
+ [Using code as ROS node](#-using-the-code-as-ros-node)

<br>

## ● Circuit
+ Raspberry pi cannot get input for 5V but 3.3V so we have to reduce voltage using Resistance like [here](https://blog.naver.com/roboholic84/220319850312)
<p align="center">
<img src="https://github.com/engcang/image-files/blob/master/sonar_sensor/Resistance.jpg" width="500" hspace="0"/>
</p>
<br>

+ Plug into GPIO, for my code, 
  + Vcc to any 5V (pin number 2)
  + GND to any Ground (pin number 9)
  + TRIG to GPIO 27 (pin number 13)
  + Echo to GPIO 17 (pin number 11)
  <p align="center">
  <img src="https://github.com/engcang/image-files/blob/master/sonar_sensor/gpio.png" width="500" hspace="0"/>
  </p>

<br>

+ Result image on Raspberry pi board on Turtlebot3

<p align="center">
<img src="https://github.com/engcang/image-files/blob/master/sonar_sensor/Raspberry.jpg" width="500" hspace="0"/>
</p>

</br></br>

## ● Code explanation
+ Import libraries and setup GPIO pins
  ~~~python
  #!/usr/bin/env python
  import RPi.GPIO as gpio
  import time
  import sys
  import signal

  def signal_handler(signal, frame): # ctrl + c -> exit program
          print('You pressed Ctrl+C!')
          sys.exit(0)
  signal.signal(signal.SIGINT, signal_handler)

  gpio.setmode(gpio.BCM)
  trig = 27 # 7th
  echo = 17 # 6th

  gpio.setup(trig, gpio.OUT)
  gpio.setup(echo, gpio.IN)
  ~~~
  1.**signal_handler** function exit program when terminal gets **CTRL + C** <br>
  2.You should change **trig** and **echo** value if you want to plug those into different GPIOs <br>

<br>

+ Trig pulse and wait Echo
  ~~~python
  time.sleep(0.5)
  print ('-----------------------------------------------------------------sonar start')
  try :
      while True :
          gpio.output(trig, False)
          time.sleep(0.1)
          gpio.output(trig, True)
          time.sleep(0.00001)
          gpio.output(trig, False)
          while gpio.input(echo) == 0 :
              pulse_start = time.time()
          while gpio.input(echo) == 1 :
              pulse_end = time.time()
          pulse_duration = pulse_end - pulse_start
          distance = pulse_duration * 17000
          if pulse_duration >=0.01746:
              print('time out')
              continue
          elif distance > 300 or distance==0:
              print('out of range')
              continue
          distance = round(distance, 3)
          print ('Distance : %f cm'%distance)

  except (KeyboardInterrupt, SystemExit):
      gpio.cleanup()
      sys.exit(0)
  except:
      gpio.cleanup()
  ~~~
  3.Wait 0.1 second to be stable and then Trig pulse for 0.00001 sec <br>
  4.Wait until Echo pulse comes in and then calculate pulse_duration time <br>
  5.Calculate Distance using **Speed of Sound**, approximately 340m/s, here 17000cm/s for round-trip of pulse <br>
  6.Limit the result by time out and maximum distance by sensor's specification <br>

<br>

## ● Using the code directly
+ git clone the codes first
  ~~~shell
  $ git clone https://github.com/engcang/HC-SR04-UltraSonicSensor-ROS-RaspberryPi.git
  $ cd HC-SR04-UltraSonicSensor-ROS-RaspberryPi
  $ python sonar_sensor.py
  ~~~
<br>

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
  Simply add this line into launch file you want to launch together

<br>

+ Result data by _**rostopic echo /sonar_dist**_
<p align="center">
<img src="https://github.com/engcang/image-files/blob/master/sonar_sensor/ROS_topic.gif" width="400" height="500" hspace="0"/>
</p>
