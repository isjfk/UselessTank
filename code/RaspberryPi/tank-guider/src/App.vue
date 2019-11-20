<template>
  <div id="app">
    <main>
      <router-view></router-view>
      <!-- img tag must be placed after previous div, otherwise the div will not overlapping over img -->
      <img id="map" :src="mapUrl" alt="Oops... Where is my map?" />
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
    display: flex;
    align-items: center;
    justify-content: center;
    height: 100vh;
  }

  main {
    position: relative;
    z-index: 0;
    height: 100%;
    width: fit-content;
    overflow-y: hidden;
  }

  #map {
    height: 100%;
  }
</style>
