<template>
  <div id="markLayer" :style="{ width: map.screenWidth+'px', height: map.screenHeight+'px' }">
    <svg id="tankPathLayer">
      <line class="tankPath" v-for="tankPath in tankPathList" :x1="tankPath.x1+'%'" :y1="tankPath.y1+'%'" :x2="tankPath.x2+'%'" :y2="tankPath.y2+'%'" />
    </svg>
    <div id="poiLayer">
      <div class="poi" v-for="poi in poiList" :style="{ left: poi.x+'%', top: poi.y+'%' }">
        <span>{{poi.name}}</span>
      </div>
    </div>
    <img id="tank" src="../assets/leopard2.png" alt="Oops... The tank is gone." :style="{ left: tankPosition.x+'%', top: tankPosition.y+'%', transform: 'translate(-50%, -50%) rotate('+tankPosition.yaw+'deg)' }" />
    <div id="gotoLayer" @click="tankGoto($event)" />
  </div>
</template>

<script>
export default {
  name: 'location',
  data () {
    return {
      map: { pixWidth: 1, pixHeight: 1, screenWidth: 1, screenHeight: 1, resolution: 0.05, scale: 1 },
      tankImg: { screenWidth: 1, screenHeight: 1, xOffset: 0, yOffset: 0 },
      tankPosition: { x: -1, y: -1, yaw: 0 },
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

    tankGoto (event) {
      let x = event.offsetX / this.map.screenWidth * this.map.pixWidth * this.map.resolution
      let y = (this.map.screenHeight - event.offsetY) / this.map.screenHeight * this.map.pixHeight * this.map.resolution

      let that = this
      this.$axios.post(this.rosRestUrl() + '/tank/action/goto', {
        x: x,
        y: y
      }).then(function (response) {
        that.getTankPath()
      })
    },

    getPoiList () {
      var that = this
      this.$axios.get(this.rosRestUrl() + '/poi').then(function (response) {
        let poiList = response.data

        for (let i = 0; i < poiList.length; i++) {
          let poi = poiList[i]
          poi.x = (poi.x / that.map.resolution) / that.map.pixWidth * 100.0
          poi.y = (that.map.pixHeight - (poi.y / that.map.resolution)) / that.map.pixHeight * 100.0
        }

        that.poiList = poiList

        console.log(that.poiList)
      })
    },

    getTankPosition () {
      var that = this
      this.$axios.get(this.rosRestUrl() + '/tank/position').then(function (response) {
        let tankPos = response.data

        tankPos.x = (tankPos.x / that.map.resolution) / that.map.pixWidth * 100.0
        tankPos.y = (that.map.pixHeight - (tankPos.y / that.map.resolution)) / that.map.pixHeight * 100.0
        tankPos.yaw = -tankPos.yaw

        that.tankPosition = tankPos
        if (that.tankPosition.x >= 0) {
          document.getElementById('tank').style.visibility = 'visible'
        }

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
      this.refreshMapImgSize()
      this.refreshTankImgSize()
    },

    tankRefreshLoop () {
      this.refreshMapImgSize()
      this.refreshTankImgSize()

      this.getTankPath()
      this.getTankPosition()
    },

    poiRefreshLoop () {
      this.refreshMapImgSize()

      this.getPoiList()
    },

    refreshMapImgSize () {
      let mapImg = document.getElementById('map')
      // Sometimes image width/height is e.g. 640.7, which will become 641 when get from element.
      // So we subtract 1 to prevent markLayer large then the map.
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
      this.tankImg.screenWidth = tankImg.width
      this.tankImg.screenHeight = tankImg.height
      this.tankImg.xOffset = -(tankImg.width / 2)
      this.tankImg.yOffset = -(tankImg.height / 2)
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
      this.tankRefreshLoop()
    }, 500)

    setTimeout(() => {
      this.poiRefreshLoop()
    }, 500)
    setInterval(() => {
      this.poiRefreshLoop()
    }, 5000)
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
  }

  #tankPathLayer {
    width: 100%;
    height: 100%;
  }

  .tankPath {
    stroke: rgb(0, 255, 0);
    stroke-width: 2;
  }

  #poiLayer {
    position: absolute;
    width: 100%;
    height: 100%;
    left: 0px;
    top: 0px;
  }

  .poi {
    position: absolute;
    transform: translate(-50%, -50%);
    padding: 0.5vh;
    background-color: orange;
    border-color: chocolate;
    border-style: solid;
    text-align: center;
    font-size: 2vh;
    font-weight: bold;
  }

  #tank {
    position: absolute;
    width: auto;
    height: 6%;
    visibility: hidden;
  }

  #gotoLayer {
    position: absolute;
    width: 100%;
    height: 100%;
    left: 0px;
    top: 0px;
  }
</style>
