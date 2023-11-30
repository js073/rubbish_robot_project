<!-- eslint-disable -->
<template>
  <div class="robot-dashboard">
    <h2>Robot Dashboard</h2>
    <div>Status: {{ robotStatus }}</div>
    <div>Location: {{ robotLocation.x }}, {{ robotLocation.y }}</div>
    <div>Path: {{ robotPath }}</div>
    <div>Battery Level: {{ batteryLevel }}%</div>
    <div>Sensor Status: {{ sensorStatus }}</div>

    <!-- New Section: Camera View -->
    <div class="camera-view">
      <h3>Camera View</h3>
      <!-- Add your camera view implementation here -->
      <!-- For example, you can display an image or video stream -->
      <!-- <img :src="cameraImageUrl" alt="Camera View" /> -->
    </div>

    <!-- Buttons for controlling the robot -->
    <!-- <button @click="startRobotMapping" :disabled="robotStatus !== 'Idle'">Start Mapping</button> -->
    <button @click="startRobotMapping">Start Mapping</button>
    <button @click="stopRobotMapping">Stop Mapping</button>
    <button @click="startRobotTask">Start Task</button>
    <button @click="stopRobotTask">Stop Task</button>
  </div>
</template>

<script>
import ROSLIB from 'roslib';

const ros = new ROSLIB.Ros({
        url: 'ws://localhost:9000',
});

const robotControl = new ROSLIB.Topic({
  ros,
  name: '/main/commands',
  messageType: 'std_msgs/String', // i would adjust the message type as needed
});

var robotInfo = new ROSLIB.Topic({
  ros, 
  name: '/main/info',
  messageType: 'std_msgs/String',
});

robotInfo.subscribe(function(message) {
  const str = message.data;
  // change the status
});

export default {
  data() {
    return {
      robotStatus: 'Idle',
      robotLocation: { x: 0, y: 0 },
      robotPath: [],
      batteryLevel: 75, // Example battery level
      sensorStatus: 'Normal',
      cameraImageUrl: 'path_to_camera_image.jpg', // Example camera image URL
    };
  },
  methods: {
    startRobotMapping() {
      // Call your ROS service or emit a message to start the robot
      this.publishToROS("map_start|test1");
    },
    stopRobotMapping() {
      // Call your ROS service or emit a message to stop the robot
      this.publishToROS("map_end");
    },
    startRobotTask() {
      this.publishToROS("collection_start");
    },
    stopRobotTask() {
      this.publishToROS("collection_end");
    },
    updateStatus(message) {
      this.robotStatus = message;
    },
    publishToROS(message) {
      // I would replace the ROS server URL with your actual ROS server UR

      const robotMessage = new ROSLIB.Message({
        data: message,
      });

      robotControl.publish(robotMessage);
    },
  },
};
</script>

<style scoped>
.robot-dashboard {
  padding: 16px;
  border: 1px solid #ccc;
  margin-bottom: 16px;
}

.camera-view {
  margin-top: 20px;
}
</style>
