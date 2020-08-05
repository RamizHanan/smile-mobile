import ROSLIB from "roslib"; //Library providing all the connections with ROS.

export default {
  data: function() {
    return {
      roll: "N/A",
      pitch: "N/A",
      yaw: "N/A",
      drawer: null,
      ros: null,
      connected: false, //Are we connected to robot?
      camera: "./assets/logo.png",
      imu_subscriber: null, //Only connect when successful connection is established
      image_topic: null,
      pwm_topic: null,
      pwm_message: null,
      joy_publisher: null,
      joy_topic: null,
      joy_message: null,
      left_pwm_raw: 0,
      right_pwm_raw: 0,
      left_pwm: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
      right_pwm: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],

      motorsEnabled: true,
      motors_enable_request_message: null,
      motors_enable_service: null,
    };
  },

  methods: {
    //Connect to a ROS websocket to communicate with selected vehicle
    connect: function(url) {
      this.ros = new ROSLIB.Ros({
        url: url,
      });

      this.ros.on("connection", function() {
        console.log("Connected to websocket server.");
        this.connected = true;
      });

      console.log(this.connected);
      this.ros.on("error", function(error) {
        console.log("Error connecting to websocket server: ", error);
        this.connected = false;
      });

      this.ros.on("close", function() {
        console.log("Connection to websocket server closed.");
        this.connected = false;
      });

      //Connect to the IMU data (Odometry)
      this.imu_subscriber = new ROSLIB.Topic({
        ros: this.ros,
        name: "/smile/imu",
        messageType: "sensor_msgs/Imu",
      });

      this.imu_subscriber.subscribe(this.imu_subscriber_callback);

      this.joy_publisher = new ROSLIB.Topic({
        ros: this.ros,
        name: "/smile/joy",
        messageType: "sensor_msgs/Joy",
      });

      this.joy_message = new ROSLIB.Message({axes : [], buttons : []});
      //
      this.image_topic = new ROSLIB.Topic({
        ros: this.ros,
        name: "/smile/cv2_capture",
        messageType: "sensor_msgs/CompressedImage",
      });

      this.image_topic.subscribe(this.image_topic_callback);

      this.pwm_topic = new ROSLIB.Topic({
        ros: this.ros,
        name: "/smile/pwm",
        messageType: "std_msgs/Int16MultiArray",
      });

      this.pwm_message = new ROSLIB.Message({
        data: [1, 2, 3, 4],
      });

      this.motors_enable_service = new ROSLIB.Service({
        ros: this.ros,
        name: "/smile/motors_enabled",
        serviceType: "std_srvs/SetBool",
      });

      this.motors_enable_request_message = new ROSLIB.ServiceRequest({
        data: true,
      });
    },

    imu_subscriber_callback: function(message) {
      this.roll = (message.orientation.x * (180.0 / Math.PI))
        .toFixed(2)
        .toString();
      this.pitch = (message.orientation.y * (180.0 / Math.PI))
        .toFixed(2)
        .toString();
      this.yaw = (message.orientation.z * (180.0 / Math.PI))
        .toFixed(2)
        .toString();
    },

    image_topic_callback: function(message) {
      this.camera = "data:image/jpg;base64," + message.data;
    },

    motor_enable_service_response_callback: function(response) {
      console.log(response);
    },

    toggleMotorState: function() {
      this.motorsEnabled = !this.motorsEnabled;
      this.motors_enable_request_message.data = this.motorsEnabled;
      this.motors_enable_service.callService(
        this.motors_enable_request_message,
        this.motor_enable_service_response_callback
      );

      console.log("Setting Motor State to " + this.motorsEnabled);
    },
  },
};
