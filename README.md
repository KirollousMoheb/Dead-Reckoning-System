# Dead Reckoning System
# Description

> This System was designed for the FSUK ASU Racing Team Vehicle, for drawing the vehicle's path on the Silverstone Track where each point on the drawn path carries information from various sensors regarding the vehicle's state.
 
 ## Table of Contents

- [Hardware](#hardware)
  * [STM32F103](#sTM32F103)
  * [NEO-M8P](#NEO-M8P)
- [Tasks](#tasks)
- [Output](#output)


## Hardware
### STM32F103
 <p align="center">
&nbsp; &nbsp; &nbsp; &nbsp;
  <img alt="Dark" src="https://alselectro.files.wordpress.com/2018/11/stm32_ok2.jpg" width="45%">
</p>

### NEO-M8P
 The NEO-M8P-2 module is great module for high accuracy GNSS and GPS location solutions including RTK.The NEO-M8P-2 is unique in that it has four communication ports which are all active simultaneously. You can read NMEA data over I2C while you send configuration commands over the UART and vice/versa. The only limit is that the SPI pins are mapped onto the I2C and UART pins so it’s either SPI or I2C+UART. The USB port is available at all times. <p align="center">
  <img alt="Light" src="https://cdn.sparkfun.com//assets/parts/1/3/3/2/0/15005-SparkFun_GPS-RTK__Qwiic__-_NEO-M8P-2-00.jpg" width="45%">
&nbsp; &nbsp; &nbsp; &nbsp;
</p>


## Tasks

| Task     | Execution time   | Relative Deadline  | Periodicity  | Priority
| ------------- |:----------:| -----:| -----:|-----:|
| GPS Task | 1ms | 100ms | 100ms |2|
| CAN Task | 5ms | 100ms | 100ms |1|

*all times are in ticks set at 1kHz, therefore 1000 ticks = 1 second*


## Output

<p align="center">
  <img alt="Light" src="https://user-images.githubusercontent.com/66686446/183115027-b4b06ece-7ec9-480e-9d3a-bee0af69f1a6.jpg" width="45%">
&nbsp; &nbsp; &nbsp; &nbsp;
  <img alt="Dark" src="https://user-images.githubusercontent.com/66686446/183115030-10632370-2690-4ff4-8724-797c5e3f7258.jpg" width="45%">
</p>

![Screenshot (1425)](https://user-images.githubusercontent.com/66686446/183111821-e6ab1b63-d017-4fcb-b54e-e6a8fdb3167c.png)


<!-- ![COM6_-_PuTTY_2022-01-02_18-19-58_AdobeExpress](https://user-images.githubusercontent.com/66686446/183113172-9c8e0bee-57af-4b97-8660-c7f3f0268c2f.gif)



<a href="url"><img src="https://user-images.githubusercontent.com/66686446/183113172-9c8e0bee-57af-4b97-8660-c7f3f0268c2f.gif" height="500" width="500" ></a> -->
