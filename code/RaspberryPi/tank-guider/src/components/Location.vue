<template>
  <div id="markLayer" :style="{ width: map.screenWidth+'px', height: map.screenHeight+'px' }">
    <svg class="markSvg">
      <line class="tankPath" v-for="tankPath in tankPathList" :x1="tankPath.x1+'%'" :y1="tankPath.y1+'%'" :x2="tankPath.x2+'%'" :y2="tankPath.y2+'%'" />
      <image id="tank" xlink:href="../assets/leopard2.png" :x="tankPosition.x+tankImg.xOffset" :y="tankPosition.y+tankImg.yOffset" :transform="'rotate(' + tankPosition.yaw + ',' + tankPosition.x + ',' + tankPosition.y + ')'" width="10%" height="10%" preserveAspectRatio="xMidYMid meet" />
    </svg>
    <div v-for="(poi, $index) in poiList">
      <div v-bind:style="{position: 'absolute', left: poi.x + '%', top: poi.y + '%'}" v-bind:id="room.id" v-bind:x="room.x" v-bind:y="room.y" @click="go(room)">
        <span class="location">{{room.name}}</span>
      </div>
    </div>
  </div>
</template>

<script>
export default {
  name: 'location',
  data () {
    return {
      map: { pixWidth: 1, pixHeight: 1, screenWidth: 1, screenHeight: 1, resolution: 0.05, scale: 1 },
      tankImg: { screenWidth: 1, screenHeight: 1, xOffset: 0, yOffset: 0 },
      tankPosition: { x: 0, y: 0, yaw: 0 },
      tankPathList: [],
      poiList: []
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
      let x = null
      let y = null
      this.$axios.get(this.rosRestUrl() + '/tank/action/goto?x=' + x + '&y=' + y).then(function (response) {
      })
    },

    getTankPosition () {
      var that = this
      this.$axios.get(this.rosRestUrl() + '/tank/position').then(function (response) {
        let tankPos = response.data

        tankPos.x = (tankPos.x / that.map.resolution) * that.map.scale
        tankPos.y = (that.map.pixHeight - (tankPos.y / that.map.resolution)) * that.map.scale
        tankPos.yaw = -tankPos.yaw

        that.tankPosition = tankPos
        that.refreshTankImgSize()

        // console.log(that.tankPosition)
      })
    },

    getTankPath () {
      var that = this
      this.$axios.get(this.rosRestUrl() + '/tank/path').then(function (response) {
        let tankPathData = response.data
        let tankPathList = []

        if (tankPathData.length > 1) {
          let prevX = tankPathData[0].x / that.map.resolution / that.map.pixWidth * 100.0
          let prevY = (that.map.pixHeight - (tankPathData[0].y / that.map.resolution)) / that.map.pixHeight * 100.0

          for (let i = 1; i < tankPathData.length; i++) {
            let line = {}
            line.x1 = prevX
            line.y1 = prevY
            line.x2 = tankPathData[i].x / that.map.resolution / that.map.pixWidth * 100.0
            line.y2 = (that.map.pixHeight - (tankPathData[i].y / that.map.resolution)) / that.map.pixHeight * 100.0

            prevX = line.x2
            prevY = line.y2

            tankPathList.push(line)
          }

          // console.log(tankPathList)
        }

        that.tankPathList = tankPathList
      })
    },

    onWindowResize () {
      // Temporary hidden the mark layer because of map scale changed.
      document.getElementById('markLayer').style.visibility = 'hidden'
    },

    refreshLoop () {
      this.refreshMapImgSize()

      this.getTankPath()
      this.getTankPosition()
    },

    refreshMapImgSize () {
      let mapImg = document.getElementById('map')
      // Sometimes image size is 1 pix large then actual float size in screen
      this.map.screenWidth = mapImg.width - 1
      this.map.screenHeight = mapImg.height - 1

      let tmpImg = new Image()
      tmpImg.src = mapImg.src
      this.map.pixWidth = tmpImg.width
      this.map.pixHeight = tmpImg.height
      this.map.scale = mapImg.width / tmpImg.width
    },

    refreshTankImgSize () {
      let tankImg = document.getElementById('tank')
      this.tankImg.screenWidth = tankImg.width.baseVal.value
      this.tankImg.screenHeight = tankImg.height.baseVal.value
      this.tankImg.xOffset = -(tankImg.width.baseVal.value / 2)
      this.tankImg.yOffset = -(tankImg.height.baseVal.value / 2)

      if (this.tankImg.screenWidth > 1) {
        document.getElementById('markLayer').style.visibility = 'visible'
      }
    }

  },

  created () {
    window.addEventListener('resize', this.onWindowResize)
  },

  destroyed () {
    window.removeEventListener('resize', this.onWindowResize)
  },

  mounted () {
    setInterval(() => {
      this.refreshLoop()
    }, 500)
  }

}
</script>

<!-- Add "scoped" attribute to limit CSS to this component only -->
<style scoped rel="stylesheet/less" lang="less">
  #markLayer {
    position: absolute;
    z-index: 1;
    height: 100%;
    width: 100%;
    overflow: hidden;
    visibility: hidden;
  }

  .markSvg {
    width: 100%;
    height: 100%;
  }

  .tankPath {
    stroke: rgb(0, 255, 0);
    stroke-width: 2;
  }

  .location {
    border-color: chocolate;
    background-color: orange;
    border-style: solid;
    font-size: x-large;
    text-align: center;
    font-weight: bold;
  }
</style>
