<template>
  <v-app id="inspire">
    <v-app-bar app clipped-left>
      <v-toolbar-title>SMILE Robotics</v-toolbar-title>
    </v-app-bar>

    <v-main>
      <v-container fluid>
        <v-row>
          <v-col>
            <OdometryDisplay
              v-bind:roll="roll"
              v-bind:pitch="pitch"
              v-bind:yaw="yaw"
            />
          </v-col>
          <v-col>
            <img :src="camera" width="600" height="400" />
          </v-col>
        </v-row>
        <v-row>
          <v-col class="text-center">
            <v-btn
              large
              v-on:click="toggleMotorState"
              color="error"
              v-if="motorsEnabled"
              >Disable Motors</v-btn
            >
            <v-btn large v-on:click="toggleMotorState" color="success" v-else
              >Enable Motors</v-btn
            >
          </v-col>
        </v-row>
      </v-container>
      <WASDTeleop v-on:wasdCommand="receiveWASD" />
      <Joystick v-on:joyCommand="receiveJOY" />
    </v-main>

    <v-footer app>
      <span>&copy; {{ new Date().getFullYear() }}</span>
    </v-footer>
  </v-app>
</template>

<script>
import OdometryDisplay from "@/components/OdometryDisplay";
import ros_mixin from "./ros_mixin.js";
import WASDTeleop from "@/components/WASDTeleop/WASDTeleop";
import Joystick from "@/components/Joystick/Joystick";

export default {
  props: {
    source: String,
  },

  /*
    ros_mixin contains the connections with getting and receiving information
    from ros.
    */
  mixins: [ros_mixin],

  components: {
    OdometryDisplay,
    WASDTeleop,
    Joystick,
  },

  methods: {
    receiveWASD: function(W, A, S, D) {
      console.log("-----------------------");
      this.left_pwm_raw = W * 50 + S * -50 + A * -40 + D * 40;
      this.right_pwm_raw = W * 50 + S * -50 + A * 40 + D * -40;
      this.left_pwm.shift();
      this.left_pwm.push(this.left_pwm_raw);
      this.right_pwm.shift();
      this.right_pwm.push(this.right_pwm_raw);

      // let left_sum = 0;
      // let right_sum = 0;
      // for (let i = 0; i < 10; i++) {
      //   left_sum += this.left_pwm[i];
      //   right_sum += this.right_pwm[i];
      // }
      // left_sum = left_sum / 10.0;
      // right_sum = right_sum / 10.0;

      // this.pwm_message.data = [left_sum, right_sum, right_sum, left_sum];
      // console.log(this.pwm_message.data);
      // this.pwm_topic.publish(this.pwm_message);
    },
    receiveJOY: function(buttons, axes) {
      
      var FWBW_pwmFL = -1 * axes[1] * 75;
      var FWBW_pwmBL = -1 * axes[1] * 75;
      var FWBW_pwmFR = -1 * axes[1] * 75;
      var FWBW_pwmBR = -1 * axes[1] * 75;

      var LR_pwmFL = 0;
      var LR_pwmBL = 0;
      var LR_pwmFR = 0;
      var LR_pwmBR = 0;

      if (axes[2] >= 0.0) {
        LR_pwmFR = 0;
        LR_pwmBR = 0;
        LR_pwmFL = axes[2] * 50;
        LR_pwmBL = axes[2] * 50;
      } else {
        LR_pwmFR = 0 - axes[2] * 50;
        LR_pwmBR = 0 - axes[2] * 50;
        LR_pwmFL = 0;
        LR_pwmBL = 0;
      }
      var total_pwmFL = FWBW_pwmFL + LR_pwmFL;
      var total_pwmBL = FWBW_pwmBL + LR_pwmBL;
      var total_pwmFR = FWBW_pwmFR + LR_pwmFR;
      var total_pwmBR = FWBW_pwmBR + LR_pwmBR;

      if (buttons[1]) {
        total_pwmFL = 0;
        total_pwmBL = 0;
        total_pwmFR = 0;
        total_pwmBR = 0;
      }
      if (buttons[5]) {
        total_pwmFR = -100;
        total_pwmBR = -100;
        total_pwmFL = 100;
        total_pwmBL = 100;
      }
      if (buttons[4]) {
        total_pwmFR = 100;
        total_pwmBR = 100;
        total_pwmFL = -100;
        total_pwmBL = -100;
      }

      this.pwm_message.data = [total_pwmFL | 0, total_pwmFR | 0, total_pwmBR | 0, total_pwmBL | 0];
      console.log(this.pwm_message.data);
      this.pwm_topic.publish(this.pwm_message);
      // console.log(buttons);
      // console.log(axes);
    },
  },

  created() {
    this.$vuetify.theme.dark = true;
  },

  mounted() {
    this.connect("ws://192.168.1.207:9090");
  },
};
</script>
