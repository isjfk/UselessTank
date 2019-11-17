<template>
  <div>
    <div v-bind:style="{position: 'absolute',top: tankPosition.x + 'px', left: tankPosition.y + 'px'}" v-bind:x="tankPosition.x" v-bind:y="tankPosition.y"> <img id="tank" src="../assets/tank.png" alt="Vue.js PWA" /></div>
    <div v-bind:style="{position: 'absolute',top: home.x + 'px', left: home.y + 'px'}" v-bind:id="home.id" v-bind:x="home.x" v-bind:y="home.y" @click="go(home)">
        <span class="location">{{home.name}}</span>
       </div>
    <div v-for="(room, $index) in rooms">
       <div v-bind:style="{position: 'absolute',top: room.x + 'px', left: room.y + 'px'}" v-bind:id="room.id" v-bind:x="room.x" v-bind:y="room.y" @click="go(room)">
        <span class="location">{{room.name}}</span>
       </div>
    </div>
   <div v-for="(route, $index) in tankPath">       
       <div v-bind:style="{position: 'absolute',top: route.x + 'px', left: route.y + 'px'}" v-bind:id="route.id" v-bind:x="route.x" v-bind:y="route.y">
        <span class="route">{{route.x}}</span>
       </div>
    </div>
  </div>
</template>

<script>
import {jsPlumb} from 'jsplumb'
export default {
  name: 'location',
  data () {
    return {
      proportion: 1,
      tankPosition: {x: '190', y: '330'},
      home: {x: '190', y: '330'},
      rooms: [
        {name: 'Home', x: '220', y: '580'},
        { name: 'Meeting Room 3.04', x: '400', y: '580', id: 'r1' },
        { name: 'Meeting Room 3.06', x: '220', y: '980', id: 'r2' },
        { name: 'Meeting Room 3.05', x: '460', y: '980', id: 'r3' }
      ],
      tankPath: []
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

    go (room) {
      console.log('go to room', room.x)
      var target = room.x / this.proportion + ',' + room.y / this.proportion
      var that = this
      this.$axios.get(this.rosRestUrl() + '/tank/action/goto?target=' + target).then(function (response) {
        that.tankPath = response.data
        that.tankPath.forEach(route => {
          route.x = route.x * that.proportion
          route.y = route.y * that.proportion
        })
      })
    },

    drawTankPath () {
      console.log('draw route')
      let plumbIns = jsPlumb.getInstance()
      var that = this
      plumbIns.ready(function () {
        plumbIns.deleteEveryConnection()
        for (let i = 0; i < that.tankPath.length - 1; i++) {
          plumbIns.connect({
            source: that.tankPath[i].id,
            target: that.tankPath[i + 1].id,
            anchor: ['Left', 'Right', 'Top', 'Bottom', [0.3, 0, 0, -1], [0.7, 0, 0, -1], [0.3, 1, 0, 1], [0.7, 1, 0, 1]],
            connector: ['StateMachine'],
            endpoint: 'Blank',
            overlays: [ ['Arrow', { width: 8, length: 8, location: 1 }] ],
            paintStyle: { stroke: '#909399', strokeWidth: 2 }
          })
        }
      })
    },

    getTankPosition () {
      var that = this
      this.$axios.get(this.rosRestUrl() + '/tank/position').then(function (response) {
        let tankPos = response.data
        that.tankPosition.x = tankPos.x * that.proportion
        that.tankPosition.y = tankPos.y * that.proportion
      })
    },

    getTankPath () {
      var that = this
      this.$axios.get(this.rosRestUrl() + '/tank/path').then(function (response) {
        that.tankPath = response.data
        that.drawTankPath()
      })
    },

    refreshLoop () {
      this.getTankPosition()
      this.getTankPath()
      this.drawTankPath()
    }

  },

  created () {

  },

  destroyed () {

  },

  updated () {
    this.drawTankPath()
  },

  mounted () {
    var that = this
    var img = document.getElementById('map')
    var image = new Image()
    image.src = img.src
    this.proportion = img.width / image.width

    this.$axios.get(this.rosRestUrl() + '/locations').then(function (response) {
      that.home = response.data.home
      that.rooms = response.data.rooms

      that.home.x = that.home.x * that.proportion
      that.home.y = that.home.y * that.proportion

      that.rooms.forEach(room => {
        room.x = room.x * that.proportion
        room.y = room.y * that.proportion
      })
    })

    this.refreshLoop()
    setInterval(() => {
      this.refreshLoop()
    }, 500
    )
  }

}
</script>

<!-- Add "scoped" attribute to limit CSS to this component only -->
<style scoped rel="stylesheet/less" lang="less">
   .location{         
    border-color: chocolate;
    background-color: orange;  
    border-style: solid;
    font-size: x-large;
    text-align: center;
    font-weight: bold;      
   }

   .route{         
    display: none;     
   }

   #tank{
     width:15%
   }
</style>
