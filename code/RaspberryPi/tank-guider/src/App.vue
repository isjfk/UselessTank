<template>
  <div id="app">
    <main>
      <router-view></router-view>
      <!-- img tag must be placed after previous div, otherwise the div will not overlapping over img -->
      <img id="map" :src="mapUrl" alt="Oops... Where is my map?" />  
    </main>
    <div id="admin" :style="{ display:admin.display }">
      <button id="setPostion" class="adminButton" @click="tankSetPosition($event)" :style="{ background:admin.setPosition.background }">Set Position</button>
    </div>
  </div>
</template>

<script>
export default {
  name: 'app',

  data () {
    return {
      mapUrl: this.rosRestUrl() + '/map/image',
      admin: {display: 'none', setPosition: {disabled: true, background: 'indianred'}}
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

      // return 'http://10.207.99.135:5000'
    },
    tankSetPosition (event) {
      if (this.admin.setPosition.background === 'darkseagreen') {
        this.admin.setPosition.background = 'indianred'
        this.admin.setPosition.disabled = false
      } else if (this.admin.setPosition.background === 'indianred') {
        this.admin.setPosition.background = 'darkseagreen'
        this.admin.setPosition.disabled = true
      }
    }
  },

  created () {
    console.log('created')
    if (window.location.pathname.includes('admin')) {
      this.admin.display = 'flex'
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
    height: 98vh;
    padding: 1vh;
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

  #admin{
    position: absolute;
    width: 250px;
    height: 98%;
    right: 0px;
    align-items: center;
    justify-content: center;
    background: rgba(255,228,196,0.4);
  }

  .adminButton{
    height: 40px;
    width: 100%;
    background: indianred;
    font-size: x-large;
    font-weight: bold;
    color: ghostwhite;
    text-align: center;
  }
</style>
