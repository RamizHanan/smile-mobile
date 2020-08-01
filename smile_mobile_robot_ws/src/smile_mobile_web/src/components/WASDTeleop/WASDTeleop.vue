<template>
    <div>
  </div>
</template>

<script>
    export default{
        name: 'WASDTeleop',

        props: {

        },

        data : function() {
            return{
                W : false,
                A : false,
                S : false,
                D : false,
            }
        },
        
        methods : {

            onKeyDown : function(key) {
                if(key.keyCode === 87 /*w*/ && !(this.S)) {
                    this.W = true;
                }
                else if(key.keyCode === 83 /*s*/ && !(this.W)) {
                    this.S = true;
                }
                if(key.keyCode === 65 /*a*/ && !(this.D)) {
                    this.A = true;
                }
                else if(key.keyCode === 68 /*d*/ && !(this.A)) {
                    this.D = true;
                }
                this.$emit('wasdCommand', this.W, this.A, this.S, this.D)
            },

            onKeyUp : function(key) {
                if(key.keyCode === 87 /*w*/) {
                    this.W = false;
                }
                if(key.keyCode === 83 /*s*/) {
                    this.S = false;
                }
                if(key.keyCode === 65 /*a*/) {
                    this.A = false;
                }
                if(key.keyCode === 68 /*d*/) {
                    this.D = false;
                }

                this.$emit('wasdCommand', this.W, this.A, this.S, this.D)
            },

        }, 

        mounted() {
            document.addEventListener('keydown', this.onKeyDown)
            document.addEventListener('keyup', this.onKeyUp)
        },

        beforeDestroy() {
            document.removeEventListener('keydown', this.onKeyDown)
            document.removeEventListener('keyup', this.onKeyUp)
        }
    }
</script>