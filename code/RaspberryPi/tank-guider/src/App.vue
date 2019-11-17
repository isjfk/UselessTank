<template>
  <div id="app">
    <main>
      <img id="map" :src="mapUrl" alt="Vue.js PWA" />
      <router-view></router-view>
    </main>
  </div>
</template>

<script>
export default {
  name: 'app',

  data () {
    return {
      mapUrl: this.rosRestUrl() + '/map'
    }
  },

  methods: {
    rosRestUrl () {
      let origin = window.location.origin
      let portIndex = -1
      if (window.location.port) {
        portIndex = origin.indexOf(window.location.port)
      }
      if (portIndex > 0) {
        return origin.substring(0, portIndex) + '5000'
      } else {
        return origin + ':5000'
      }
    }

  }

}
</script>

<style>
body {
  margin: 0;
}

#app {
  font-family: "Avenir", Helvetica, Arial, sans-serif;
  -webkit-font-smoothing: antialiased;
  -moz-osx-font-smoothing: grayscale;
  color: #2c3e50;
}

#map {
  position: fixed;
  top: 0;
  bottom: 0;
  left: 0;
  right: 0;
  height: 100%;

  z-index: -10;
  margin: auto;
  overflow: auto;
}

main {
  text-align: center;
  margin-top: 40px;
}

header {
  margin: 0;
  height: 56px;
  padding: 0 16px 0 24px;
  background-color: #35495e;
  color: #ffffff;
}

header span {
  display: block;
  position: relative;
  font-size: 20px;
  line-height: 1;
  letter-spacing: 0.02em;
  font-weight: 400;
  box-sizing: border-box;
  padding-top: 16px;
}
</style>
