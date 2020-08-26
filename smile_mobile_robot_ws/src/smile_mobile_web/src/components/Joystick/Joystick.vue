<template>
  <div></div>
</template>

<script>
export default {
  name: "Joystick",
  props: {},
  data: function() {
    return {
      controllers: {},
      turbo: false,
      buttonValue: [],
      buttonState: [],
      axes: [],
      timestamp: Date,
      haveEvents: null,
    };
  },
  methods: {
    connecthandler: function(e) {
      this.addgamepad(e.gamepad);
    },
    addgamepad: function(gamepad) {
      this.controllers[gamepad.index] = gamepad;
      requestAnimationFrame(this.updateStatus);
    },
    disconnecthandler: function(e) {
      this.removegamepad(e.gamepad);
    },
    removegamepad: function(gamepad) {
      delete this.controllers[gamepad.index];
    },
    updateStatus: function() {
      if (!this.haveEvents) {
        this.scangamepads();
      }

      var j;

      for (j in this.controllers) {
        var controller = this.controllers[j];
        // this.buttons = controller.buttons.slice(0);
        // this.axes = controller.axes.slice(0);

        for (var i = 0; i < controller.buttons.length; i++) {
          this.buttonState[i] = controller.buttons[i].pressed;
          this.buttonValue[i] = controller.buttons[i].value;
        }

        this.axes = controller.axes.slice(0);
        // console.log("axis:" + this.axes);
        // console.log("button value:" + this.buttonValue);
      }
      var date = new Date();
      this.timestamp = date.getTime();

      requestAnimationFrame(this.updateStatus);
    },
    scangamepads: function() {
      var gamepads = navigator.getGamepads
        ? navigator.getGamepads()
        : navigator.webkitGetGamepads
        ? navigator.webkitGetGamepads()
        : [];
      for (var i = 0; i < gamepads.length; i++) {
        if (gamepads[i]) {
          if (gamepads[i].index in this.controllers) {
            this.controllers[gamepads[i].index] = gamepads[i];
          } else {
            this.addgamepad(gamepads[i]);
          }
        }
      }
    },
    updateJoystick : function(){
      if (!this.haveEvents) {
      setInterval(() => {
      this.scangamepads;
      this.$emit(
        "joyCommand",
        this.buttonValue,
        this.axes,
      );
      }, 10);
    }}
  },
  mounted() {
    this.haveEvents = "ongamepadconnected" in window;
    this.controllers = {};

    window.addEventListener("gamepadconnected", this.connecthandler);
    window.addEventListener("gamepaddisconnected", this.disconnecthandler);
    this.updateJoystick();
    
  },

  beforeDestroy() {
    window.removeEventListener("gamepadconnected", this.connecthandler);
    window.removeEventListener("gamepaddisconnected", this.disconnecthandler);
  },
};
</script>
