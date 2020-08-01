<template>
  <v-app id="inspire">
    <v-app-bar
      app
      clipped-left
    >
      
      <v-toolbar-title>SMILE Robotics</v-toolbar-title>
    </v-app-bar>

    <v-main>
      <v-container fluid>
        <v-row>
          <v-col>
            <OdometryDisplay v-bind:roll="roll" v-bind:pitch="pitch"  v-bind:yaw="yaw" />
          </v-col>
          <v-col>
            <img :src="camera"/>
          </v-col>
        </v-row>
      </v-container>
      <WASDTeleop v-on:wasdCommand="receiveWASD"/>
    </v-main>

    <v-footer app>
      <span>&copy; {{ new Date().getFullYear() }}</span>
    </v-footer>
  </v-app>
</template>



<script>
  import OdometryDisplay from '@/components/OdometryDisplay'
  import ros_mixin from './ros_mixin.js'
  import WASDTeleop from '@/components/WASDTeleop/WASDTeleop'

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
    },

    methods : {
      receiveWASD : function(W, A, S, D){
        console.log("-----------------------");

        this.left_pwm_raw = (W * 50) + (S * -50) + (A * -40) + (D * 40);
        this.right_pwm_raw = (W * 50) + (S * -50) + (A * 40) + (D * -40);

        this.left_pwm.shift(); this.left_pwm.push(this.left_pwm_raw);
        this.right_pwm.shift(); this.right_pwm.push(this.right_pwm_raw);

        
        let left_sum = 0; let right_sum = 0;
        for(let i = 0; i < 10; i++) {
          left_sum += this.left_pwm[i]; right_sum += this.right_pwm[i];

        }
        left_sum = left_sum / 10.0; right_sum = right_sum / 10.0;
        
        this.pwm_message.data = [left_sum, right_sum, right_sum, left_sum]
        console.log(this.pwm_message.data);
        this.pwm_topic.publish(this.pwm_message);
      }
    },

    created () {
      this.$vuetify.theme.dark = true;
      
    },

    mounted () {
      this.connect('ws://192.168.1.107:9090');
    },
  }
</script>