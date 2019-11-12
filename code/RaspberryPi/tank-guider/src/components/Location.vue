<template>
  <div>
    <div v-bind:style="{position: 'absolute',top: tank.x, left: tank.y}"> <img id="tank" src="../assets/tank.png" alt="Vue.js PWA" /></div>
    <div v-for="(room, $index) in rooms">       
       <div v-bind:style="{position: 'absolute',top: room.x, left: room.y}" v-bind:id="room.id" @click="go(room)">
        <span class="location">{{room.name}}</span>
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
      tank: {x: '190px', y: '330px'},
      rooms: [
        {name: 'Home', x: '220px', y: '580px'},
        { name: 'Meeting Room 3.04', x: '400px', y: '580px', id: 'r1' },
        { name: 'Meeting Room 3.06', x: '220px', y: '980px', id: 'r2' },
        { name: 'Meeting Room 3.05', x: '460px', y: '980px', id: 'r3' }
      ]
    }
  },

  methods: {
    go (room) {
      console.log('go to room')
      let plumbIns = jsPlumb.getInstance()
      plumbIns.ready(function () {
        plumbIns.deleteEveryConnection()
        plumbIns.connect({
          source: 'r1',
          target: 'r3',
          anchor: ['Left', 'Right', 'Top', 'Bottom', [0.3, 0, 0, -1], [0.7, 0, 0, -1], [0.3, 1, 0, 1], [0.7, 1, 0, 1]],
          connector: ['StateMachine'],
          endpoint: 'Blank',
          overlays: [ ['Arrow', { width: 8, length: 8, location: 1 }] ],
          paintStyle: { stroke: '#909399', strokeWidth: 2 }
        })
        plumbIns.connect({
          source: 'r3',
          target: 'r2',
          anchor: ['Left', 'Right', 'Top', 'Bottom', [0.3, 0, 0, -1], [0.7, 0, 0, -1], [0.3, 1, 0, 1], [0.7, 1, 0, 1]],
          connector: ['StateMachine'],
          endpoint: 'Blank',
          overlays: [ ['Arrow', { width: 8, length: 8, location: 1 }] ],
          paintStyle: { stroke: '#909399', strokeWidth: 2 }
        })
      })
    }
  },
  created () {

  },

  destroyed () {

  },

  mounted () {

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

   #tank{
     width:15%
   }
</style>
