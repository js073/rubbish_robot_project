<template>
  <div id="app">
    <h1>Cleaning Robot Dashboard</h1>
    <Dashboard :robotData="robotData" @startRobot="startRobot" @pauseRobot="pauseRobot" @stopRobot="stopRobot" />
  </div>
</template>

<script>
import Dashboard from './Dashboard.vue';

export default {
  components: {
    Dashboard,
  },
  data() {
    return {
      robotData: {
        status: 'Idle',
        location: { x: 0, y: 0 },
        path: [],
      },
    };
  },
  methods: {
    startRobot() {
      if (this.robotData.status === 'Idle') {
        this.robotData.status = 'Cleaning';
        // Simulate robot movement and update path
        this.simulateRobotMovement();
      }
    },
    pauseRobot() {
      if (this.robotData.status === 'Cleaning') {
        this.robotData.status = 'Paused';
      }
    },
    stopRobot() {
      if (this.robotData.status !== 'Idle') {
        this.robotData.status = 'Idle';
        this.robotData.path = [];
      }
    },
    updateStatus(message) {
      this.robotData.status = message;
    },
    simulateRobotMovement() {
      // Simulate robot movement by updating the path
      const intervalId = setInterval(() => {
        this.robotData.location.x += Math.random() * 5;
        this.robotData.location.y += Math.random() * 5;
        this.robotData.path.push({ ...this.robotData.location });

        if (this.robotData.status !== 'Cleaning') {
          clearInterval(intervalId);
        }
      }, 1000);
    },
  },
};
</script>

<style>
#app {
  font-family: Avenir, Helvetica, Arial, sans-serif;
  text-align: center;
  color: #2c3e50;
  margin-top: 60px;
}

h1 {
  font-size: 2em;
  margin-bottom: 20px;
}
</style>
