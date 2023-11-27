<!-- eslint-disable max-len -->
<template>
  <div id="app">
    <h1>Cleaning Robot Dashboard</h1>
    <Dashboard :robotData="robotData" @startRobot="startRobot" @pauseRobot="pauseRobot" @stopRobot="stopRobot" />

    <!-- New Section: Map Controls -->
    <div class="map-controls">
      <h2>Map Controls</h2>
      <button @click="startMapping" :disabled="robotData.status !== 'Idle'">Start Mapping</button>
      <button @click="endMapping" :disabled="robotData.status !== 'Idle'">End Mapping</button>
      <button @click="startCollection" :disabled="robotData.status !== 'Idle'">Start Collection</button>
      <button @click="endCollection" :disabled="robotData.status !== 'Idle'">End Collection</button>
    </div>

    <!-- New Section: Map List -->
    <div class="map-list">
      <h2>Map List</h2>
      <ul>
        <li v-for="map in mapList" :key="map">{{ map }}</li>
      </ul>
    </div>
  </div>
</template>

<script>
import Dashboard from './DashboardView.vue';

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
      mapList: [], // New data property for storing the list of maps
    };
  },
  methods: {
    startMapping() {
      // Call your ROS service or emit a message to start mapping
      this.publishToROS('/main/commands', 'map_start');
    },
    endMapping() {
      // Call your ROS service or emit a message to end mapping
      this.publishToROS('/main/commands', 'map_end');
    },
    startCollection() {
      // Call your ROS service or emit a message to start collection
      this.publishToROS('/main/commands', 'collection_start');
    },
    endCollection() {
      // Call your ROS service or emit a message to end collection
      this.publishToROS('/main/commands', 'collection_end');
    },

    // ... (existing methods)

    // New Method: Request Map List
    requestMapList() {
      this.publishToROS('/main/commands', 'map_list');
    },

    // ... (existing methods)
  },
  mounted() {
    // Fetch the map list when the component is mounted
    this.requestMapList();
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

/* New Styles */
.map-controls {
  margin-top: 20px;
}

.map-list {
  margin-top: 20px;
}
</style>
