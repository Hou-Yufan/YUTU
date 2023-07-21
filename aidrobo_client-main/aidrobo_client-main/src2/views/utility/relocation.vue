<template>
  <div class="main">
    <div>
      <img src="@/assets/img/relocation.svg" alt="relocation" />
    </div>
    <div v-if="step == '0'">
      <div class="content">请选择当前需要执行的重定位操作</div>
      <div class="btnBox">
        <div class="common" @click="step='1'">自动重定位</div>
        <div class="common">起始点复位</div>
      </div>
    </div>
    <div v-if="step == '1'">
      <div class="content">请确保机器人周围环境特征较明显后点击开始重定位，重定位过程机器人将自旋转</div>
      <div class="btnBox">
        <div class="common" @click="autoPose">开始重定位</div>
      </div>
    </div>
    <div v-if="step == '1_1'">
      <div class="content">正在尝试进行重定位，请等待完成...</div>
    </div>
  </div>
</template>

<script>
export default {
  data () {
    return {
      step: "0",
    };
  },
  methods: {
    autoPose () {
      // 重定位
      const point = {
        header: {
          stamp: {
            sec: 0,
            nanosec: 0
          },
          frame_id: "map"
        },
        pose: {
          position: {
            x: 0.0,
            y: 0.0,
            z: 0.0
          },
          orientation: {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 1.0
          }
        }
      };
      var pose_msg = new ROSLIB.Message(point);
      PoseStamped.publish(pose_msg);
    }
  },
}
</script>

<style lang="less" scoped>
.main {
  display: flex;
  flex-direction: column;
  align-items: center;
  margin-top: 60px;
}

.btnBox {
  position: absolute;
  bottom: 140px;
  width: 100%;
  display: flex;
  justify-content: space-evenly;
  left: 0;

  .common {
    width: 380px;
    height: 120px;
    border-radius: 10px;
    background: linear-gradient(106deg, #596AB5 10%, rgba(66, 82, 146, 0.53) 89%);
    backdrop-filter: blur(10.88px);
    box-shadow: 0px 2px 10px 0px rgba(1, 29, 90, 0.72);
    text-align: center;
    line-height: 120px
  }
}
</style>
