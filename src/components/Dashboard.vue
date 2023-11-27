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
    <button @click="startRobot" :disabled="robotStatus !== 'Idle'">Start Robot</button>
    <button @click="pauseRobot" :disabled="robotStatus !== 'Cleaning'">Pause Robot</button>
    <button @click="stopRobot" :disabled="robotStatus === 'Idle'">Stop Robot</button>
  </div>
</template>

<script>
import ROSLIB from 'roslib';

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
    startRobot() {
      // Call your ROS service or emit a message to start the robot
      this.publishToROS('/start_robot', {});
    },
    pauseRobot() {
      // Call your ROS service or emit a message to pause the robot
      this.publishToROS('/pause_robot', {});
    },
    stopRobot() {
      // Call your ROS service or emit a message to stop the robot
      this.publishToROS('/stop_robot', {});
    },
    publishToROS(topic, message) {
      // I would replace the ROS server URL with your actual ROS server URL
      const ros = new ROSLIB.Ros({
        url: 'ws://localhost:9090',
      });

      const robotControl = new ROSLIB.Topic({
        ros,
        name: topic,
        messageType: 'std_msgs/String', // i would adjust the message type as needed
      });

      const robotMessage = new ROSLIB.Message({
        data: JSON.stringify(message),
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
