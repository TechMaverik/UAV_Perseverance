# UAV_Perseverance from TechMaverik
A short ranged Unmanned Areal Vehicle (Range <=100m) for Educational Purposes.

## Developers Note
The tutorial includes building a flight controller using node mcu (ESP8266) and MPU6050 (Gyroscope and Accelerometer). You will also learn about different terms like yaw, pitch, roll in this tutorial. 

## Components Required and Explaination

* NodeMCU (ESP8266) : NodeMCU is a low-cost open source IoT platform. It initially included firmware which runs on the ESP8266 Wi-Fi SoC from Espressif Systems, and hardware which was based on the ESP-12 module

* MPU6050 : MPU6050 sensor module is complete 6-axis Motion Tracking Device. It combines 3-axis Gyroscope, 3-axis Accelerometer and Digital Motion Processor all in small package. Also, it has additional feature of on-chip Temperature sensor. It has I2C bus interface to communicate with the microcontrollers.

* ESC : Electronic speed controllers (ESCs) are devices that allow drone flight controllers to control and adjust the speed of the aircraft's electric motors. A signal from the flight controller causes the ESC to raise or lower the voltage to the motor as required, thus changing the speed of the propeller.

* BLDC Motor : Quad copter drone usually uses Brushless Direct Current (BLDC) motor as rotor due to high efficiency and small volume. The BLDC motor speed control is very important for drone position and velocity determent.

* Drone Frame : A drone frame has two main parts: the body and the arms. The body houses and protects your electronic components including flight controller, 4in1 ESC, FPV camera, VTX etc. Typically it consists of a bottom plate, top plate and some standoffs in between to hold them together.

* Lipo Batteries : LiPo batteries have many benefits for the drone industry since they are small and can carry a lot of charge in one cell (3.7 V to 4.2V). This makes it possible to build a large-capacity or high voltage battery for various applications without making the drone too heavy.

## Technical Terms Familiarization

<img src="Design_Archives\Yaw_pitch_roll.png" alt="cannot display">

* <b> Yaw </b> : The yaw axis has its origin at the center of gravity and is directed towards the bottom of the aircraft, perpendicular to the wings and to the fuselage reference line. Motion about this axis is called yaw. A positive yawing motion moves the nose of the aircraft to the right.

* <b> Pitch </b> : The pitch axis (also called transverse or lateral axis) has its origin at the center of gravity and is directed to the right, parallel to a line drawn from wingtip to wingtip. Motion about this axis is called pitch. A positive pitching motion raises the nose of the aircraft and lowers the tail.

* <b> Roll </b> : The roll axis (or longitudinal axis) has its origin at the center of gravity and is directed forward, parallel to the fuselage reference line. Motion about this axis is called roll. An angular displacement about this axis is called bank. A positive rolling motion lifts the left wing and lowers the right wing. 

## Circuit Diagram
<img src="Design_Archives\Circuit_Diagram.jpg" alt="Cannot Display">

<p align ="right">(to be contined ...... )</p>
