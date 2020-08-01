import ROSLIB from 'roslib'; //Library providing all the connections with ROS.

export default {

    data : function() {
        return{
        
            roll : "N/A",
            pitch : "N/A",
            yaw : "N/A",
            drawer: null,
            ros : null,
            connected : false, //Are we connected to robot?
            camera : './assets/logo.png',
            imu_subscriber : null, //Only connect when successful connection is established
            image_topic : null,
            pwm_topic : null,
            pwm_message : null,
            left_pwm_raw : 0,
            right_pwm_raw : 0,
            left_pwm : [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            right_pwm : [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
          }
    },

    methods : {
        //Connect to a ROS websocket to communicate with selected vehicle
        connect : function(url) {
            this.ros = new ROSLIB.Ros({
                url: url
            });

            this.ros.on('connection', function() {
                console.log('Connected to websocket server.');
                this.connected = true;
            });

            console.log(this.connected);
            this.ros.on('error', function(error) {
                console.log('Error connecting to websocket server: ', error);
                this.connected = false;
            });

            this.ros.on('close', function() {
                console.log('Connection to websocket server closed.');
                this.connected = false;
            });  
            
            //Connect to the IMU data (Odometry)
            this.imu_subscriber = new ROSLIB.Topic({
                ros : this.ros,
                name : '/smile/imu',
                messageType : 'sensor_msgs/Imu'
            });

            this.imu_subscriber.subscribe(this.imu_subscriber_callback);

            //
            this.image_topic = new ROSLIB.Topic({
            ros: this.ros,
            name: '/smile/cv2_capture',
            messageType: 'sensor_msgs/CompressedImage'
            });

            this.image_topic.subscribe(this.image_topic_callback);
            
            this.pwm_topic = new ROSLIB.Topic({
                ros : this.ros,
                name : '/smile/pwm',
                messageType : 'std_msgs/Int16MultiArray',
            });

            this.pwm_message = new ROSLIB.Message({
                data: [1, 2, 3, 4],
            })
            
        },
        
        imu_subscriber_callback : function(message){
        this.roll = (message.orientation.x * (180.0 / Math.PI)).toFixed(2).toString();
        this.pitch = (message.orientation.y * (180.0 / Math.PI)).toFixed(2).toString();
        this.yaw = (message.orientation.z * (180.0 / Math.PI)).toFixed(2).toString();
        },

        image_topic_callback : function(message){
        this.camera = "data:image/jpg;base64," + message.data;
        },


        
    },
}