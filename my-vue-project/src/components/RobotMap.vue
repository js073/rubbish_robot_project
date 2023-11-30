/* eslint-disable */
<template>
  <div class="map">
    <div>Robot Path:</div>
    <ul>
      <li v-for="(point, index) in path" :key="index">{{ point.x }}, {{ point.y }}</li>
    </ul>
  </div>
</template>

<script>
export default {
  props: {
    path: Array,
  },
};
</script>

<style scoped>
.map {
  padding: 16px;
  border: 1px solid #ccc;
}
</style>
