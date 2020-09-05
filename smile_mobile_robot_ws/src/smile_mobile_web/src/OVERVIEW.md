# Vue Web Component Overview

## [main.js](https://github.com/RamizHanan/smile-mobile/blob/development/smile_mobile_robot_ws/src/smile_mobile_web/src/main.js)

Entry point for the project. This is where Vue and Vuetify are imported and instantiated. Renders App.vue.

## [ros_mixin.js](https://github.com/RamizHanan/smile-mobile/blob/development/smile_mobile_robot_ws/src/smile_mobile_web/src/ros_mixin.js)

[Vue Mixin](https://vuejs.org/v2/guide/mixins.html) was used to keep the shared ROS data in one place and use-able across all components.

Connection to ROS WebServer and creation of publishers and subscribers as well as callback functions are handled here.  

```
// Connect to the IMU data (Odometry)
  
this.imu_subscriber = new ROSLIB.Topic(
    ros: this.ros,
    name: "/smile/imu",
    messageType: "sensor_msgs/Imu",
});

this.imu_subscriber.subscribe(this.imu_subscriber_callback);
```
For full documentation you can read more about [roslibjs](http://wiki.ros.org/roslibjs/Tutorials/BasicRosFunctionality) and [mixins](https://vuejs.org/v2/guide/mixins.html).

## [App.vue](https://github.com/RamizHanan/smile-mobile/blob/development/smile_mobile_robot_ws/src/smile_mobile_web/src/App.vue)

This is the parent Vue component which serves as the main rendered page that all other components interact with. 

Currently it consists of:
* Odometry Display
* Camera Display
* WASD Keyoard Teleop
* Joystick Teleop

## [Odometry Display](https://github.com/RamizHanan/smile-mobile/blob/development/smile_mobile_robot_ws/src/smile_mobile_web/src/components/OdometryDisplay/OdometryDisplay.vue)

This component is responsible for displaying IMU data including roll, pitch, and yaw in degrees as they are captured from the sensor.
* [Insert Image here]
## Camera Display

Video footage is acquired by subscribing to the compressed image topic coming from physical camera. This can be extended to switch to simulation video by subscribing to the appropriate topic.

```
// App.vue
  
<img :src="camera" width="600" height="400" />
```
```
// rosmixin.js

// Connect to the Image data 
connect: function(url) {
    this.image_topic = new ROSLIB.Topic({
        ros: this.ros,
        name: "/smile/cv2_capture",
        messageType: "sensor_msgs/CompressedImage",
    });

    this.image_topic.subscribe(this.image_topic_callback);
},
  
image_topic_callback: function(message) {
    this.camera = "data:image/jpg;base64," + message.data;
},
```

## [WASD Teleop](https://github.com/RamizHanan/smile-mobile/blob/development/smile_mobile_robot_ws/src/smile_mobile_web/src/components/WASDTeleop/WASDTeleop.vue)
Keyboard buttons may be used to control the vehicle, although using a joystick is much easier recommended.

```
W - Forward
A - Left
S - Backward
D - Right
```
## Development 
Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change.

Please make sure to follow structure and comment as needed.