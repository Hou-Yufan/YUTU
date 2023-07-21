<template>
  <div class="newMapBox">
    <ShowMap class="map" :toolType="toolType" :initData="initData" />
    <div class="right">
      <div v-show="rubber && !stop" class="titleBox">
        <p><img src="@/assets/img/editMap/rubber.svg" />橡皮擦</p>
        <p>请使用手指或鼠标进行擦除操作，地图可使用双指或滚轮中键拖动或缩放</p>
      </div>
      <!-- <div class="rubber" @click="onRubber()" v-show="!rubber && !stop">
        <img src="@/assets/img/editMap/rubber.svg" />
        <p>橡皮擦</p>
      </div> -->
      <div v-show="!rubber && stop" class="titleBox">
        <p><img src="@/assets/img/editMap/stop.svg" />禁行线</p>
        <p>请使用手指或鼠标点击两点进行连线，地图可使用双指或滚轮中键拖动或缩放</p>
      </div>
      <div class="stop" @click="onStop()" v-show="!rubber && !stop">
        <img src="@/assets/img/editMap/stop.svg" />
        <p>禁行线</p>
      </div>
      <div class="over" @click="onOver()">{{ overText }}</div>
      <div class="out" @click="onOut()">退出</div>
    </div>
  </div>
</template>

<script>
import ShowMap from "@/components/map/edit";
import { mapState, mapMutations } from "vuex";
import { imgToMap } from "@/assets/common"

export default {
  components: {
    ShowMap
  },
  data () {
    return {
      overText: '保存地图',
      rubber: false,
      stop: false,
      initData: false,
      toolType: ''
    }
  },
  computed: {
    ...mapState([
      "linearCurveArr",
      "linearCurveArrP",
      "mapData",
    ])
  },
  mounted () {
    this.$store.state.hasSave = false;
    try {
      setTimeout(() => {
        this.getForbidden()
      }, 1000)
    } catch (error) {
    }
    // 状态机
    this.$store.state.actionStatus = 'edit'
  },
  methods: {
    DrawPicture (data) {
      const msg2 = new ROSLIB.ServiceRequest(
        {
          frame_id: "map",
          map_id: this.$route.query.id * 1,
          type: "line",
          data
        }
      );
      DrawPicture.callService(msg2, (result) => {
        if (result.success) {
          console.log('[ msg ]-75', result)
        }
        console.log('[  DrawPicture OK]-61', result)
      }, (result) => {
        console.log('[  DrawPicture ERR]-61', result)
      });
    },
    getForbidden () {
      const msg2 = new ROSLIB.ServiceRequest(
        {
          map_id: this.$route.query.id * 1,
        }
      );
      ForbiddenGet.callService(msg2, (result) => {
        if (result.success) {
          let msg = []
          try {
            console.log('result.message==>', typeof result.message)
            if (result.message.length) {
              msg = result.message
              this.initData = true
              this.$store.state.linearCurveArrP = msg
              this.hasHistory = true
            }

          } catch (error) {
            this.$message('获取禁行线失败');
          }
        } else {
          this.$message('获取禁行线失败');
        }
        console.log('[  getForbidden OK]-61', result)
      }, (result) => {
        console.log('[  getForbidden ERR]-61', result)
      });
    },
    addForbidden (data) {
      const data_ = data
      const msg2 = new ROSLIB.ServiceRequest(
        {
          map_id: this.$route.query.id * 1,
          data: JSON.stringify(data_),
          frame_id: 'map',
          data_type: 'forbidden'
        }
      );
      ForbiddenAdd.callService(msg2, (result) => {
        console.log('[  addForbidden OK]-61', result)
      }, (result) => {
        console.log('[  addForbidden ERR]-61', result)
      });
    },
    updateForbidden (data) {
      const data_ = data
      const msg2 = new ROSLIB.ServiceRequest(
        {
          id: 1,
          data: JSON.stringify({
            map_id: this.$route.query.id * 1,
            frame_id: 'map',
            point_list: JSON.stringify(data_)
          }),
          data_type: 'forbidden'
        }
      );
      ForbiddenUpdate.callService(msg2, (result) => {
        console.log('[  updateForbidden OK]-61', result)
      }, (result) => {
        console.log('[  updateForbidden ERR]-61', result)
      });
    },
    onRubber () {
      this.toolType = 'eraser'
      this.rubber = true;
      this.overText = '完成'
      this.$store.state.tool = ''
    },
    onStop () {
      this.toolType = 'stop'
      this.stop = true
      this.overText = '完成'
      this.$store.state.tool = ''
    },
    onOver () {
      if (this.overText == '保存地图') {
        console.log('[ this.linearCurveArrP. ]-158', this.linearCurveArrP)
        //禁行线
        if (this.linearCurveArrP.length) {
          if (this.hasHistory) {
            this.updateForbidden(this.linearCurveArrP)
          } else {
            this.addForbidden(this.linearCurveArrP)
          }
          this.DrawPicture(this.linearCurveArrP)
        }
      } else {
        this.toolType = ''
        this.rubber = false
        this.stop = false
        this.overText = '保存地图'
      }
    },
    onOut () {
      this.$confirm(`<div>是否确认退出</div><div>（请确认所做操作已保存）</div>`, '退出编辑', {
        dangerouslyUseHTMLString: true,
        center: true
      }).then(() => {
        this.$router.push('/map')
        console.log('[  ]-69',)
      }).catch(() => {
        console.log('[  ]-72',)
      });
    }
  }

}
</script>

<style lang="less" scoped>
.newMapBox {
  display: flex;
  color: #fff;
  width: 100%;
  height: 100%;
  position: relative;
}

.right {
  width: 434px;
  height: 1010px;
  background-color: #ccc;
  border-radius: 5px;
  background: linear-gradient(155deg, rgba(71, 84, 141, 0.64) 24%, rgba(71, 66, 124, 0.52) 98%);
  backdrop-filter: blur(10.88px);
  box-shadow: 0px 2px 31px 0px rgba(1, 29, 90, 0.72);
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: space-evenly;
  height: 1010px;
  line-height: 50px;
  margin-left: 30px;
  margin-top: 30px;

  .titleBox {
    margin-bottom: 80px;
    height: 500px;
    display: flex;
    flex-direction: column;
    text-align: center;
    justify-content: space-between;
    padding: 0 40px;
    line-height: 50px;

    p:first-child {
      display: flex;
      align-items: center;
      justify-content: space-evenly;
      font-size: 50px;

    }
  }
}

.rubber,
.stop {
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: space-evenly;
  width: 300px;
  height: 200px;
  border-radius: 20px;
  opacity: 1;
  background: linear-gradient(121deg, rgba(71, 84, 141, 0.64) 14%, rgba(53, 81, 119, 0.15) 90%, rgba(53, 92, 119, 0.14) 91%);
  backdrop-filter: blur(10.88px);
  box-shadow: 0px 2px 10px 0px rgba(1, 29, 90, 0.72);
}

.over {
  width: 300px;
  height: 120px;
  border-radius: 10px;
  opacity: 1;
  box-shadow: 0px 2px 10px 0px rgba(1, 29, 90, 0.72);
  background: #B04CF3;
  display: flex;
  align-items: center;
  justify-content: center;
}

.out {
  width: 300px;
  height: 80px;
  border-radius: 10px;
  background: #4F5478;
  opacity: 1;
  display: flex;
  align-items: center;
  justify-content: center;
  box-shadow: 0px 2px 10px 0px rgba(1, 29, 90, 0.72);
}
</style>
