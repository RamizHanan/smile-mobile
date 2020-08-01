<template>
    <div>
        <v-row>
                <v-card
                class="mx-auto"
                max-width="400"
                min-width="150"
                >
                    <v-card-text>
                    <div>Roll</div>
                    <div class="text-h5  mb-6 text-start">{{roll}} &#176;</div>
                    </v-card-text>

                    <v-card-text>
                    <div>Pitch</div>
                    <div class="text-h5  mb-6 text-start">{{pitch}} &#176;</div>
                    </v-card-text>

                    <v-card-text>
                    <div>Yaw</div>
                    <div class="text-h5  mb-6 text-start">{{yaw}} &#176;</div>
                    </v-card-text>
                </v-card>
        </v-row>
        
    
  </div>
</template>

<script>
    export default{
        name: 'OdometryDisplay',
        props: ['roll',
                'pitch',
                'yaw'],
    }
</script>