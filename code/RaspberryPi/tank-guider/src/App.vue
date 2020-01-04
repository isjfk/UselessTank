<template>
  <div id="app">
    <main>
      <router-view></router-view>
      <!-- img tag must be placed after previous div, otherwise the div will not overlapping over img -->
      <img id="map" :src="mapUrl" alt="Oops... Where is my map?" />  
    </main>
    <div id="admin" :class="{ adminEnabled: admin.isEnabled, adminDisabled: !admin.isEnabled }">
      <button id="setPosition" class="adminButton" :class="{ adminButtonPressed: admin.isSetPositionState }" @click="setPositionClicked($event)">Set Position</button>
    </div>
  </div>
</template>

<script>
import { EventBus } from './main'

export default {
  name: 'app',

  data () {
    return {
      mapUrl: this.rosRestUrl() + '/map/image',
      admin: {isEnabled: false, isSetPositionState: false}
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
    },

    setPositionClicked (event) {
      this.admin.isSetPositionState = !this.admin.isSetPositionState
      EventBus.$emit('admin.isSetPositionState', this.admin.isSetPositionState)
    }
  },

  created () {
    if (window.location.pathname.includes('admin')) {
      this.admin.isEnabled = true
    }

    EventBus.$on('admin.clearSetPositionState', () => {
      this.admin.isSetPositionState = false
      EventBus.$emit('admin.isSetPositionState', this.admin.isSetPositionState)
    })
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

  #admin {
    position: absolute;
    width: 250px;
    height: 94%;
    right: 1%;
    align-items: center;
    justify-content: center;
    padding: 1%;
    background: #69696950;
  }

  .adminDisabled {
    display: none;
  }

  .adminEnabled {
    display: flex;
  }

  .adminButton {
    height: 40px;
    width: 100%;
    background: darkseagreen;
    font-size: x-large;
    font-weight: bold;
    color: ghostwhite;
    text-align: center;
  }

  .adminButtonPressed {
    background: indianred;
  }
</style>
