# Imu data from flight controller (via [mavros](http://wiki.ros.org/mavros))

This is the node with the settings to connect the flight controller as Imu. 
As an example,was chosen a cheap flight controller cc3d.
Ideally, if you have an flight controller such as ardupilot or pixhawk you can remote contol the car via him. 
However, in cheap flight controllers, the mavlink serves only to send data to the OSD, so you can only read the data with them.
![](https://img3.banggood.com/thumb/gallery/upload/2014/10/SKU153975-94.jpg)

## Setting
Through GCS adjust telemetry through mavlink. 
Choose any uhart that connect to Rpi. <br/>

```
RX -> TX
TX -> RX
Gnd -> Gnd
```
Next setup port in cc3d.launch file and run.

You will see the data in the topics
