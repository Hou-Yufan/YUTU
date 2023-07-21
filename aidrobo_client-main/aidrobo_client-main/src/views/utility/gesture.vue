<template>
  <div class="gesture-container">
    <div class="video-container" id="video"></div>
    <div class="right">
      <div class="rText">
        <p class="title">手势识别</p>
        <p class="description">请在摄像头摄像范围内招手召唤机器人</p>
      </div>
      <div class="rLogo">
        <img src="@/assets/img/gesture.svg" />
        <p>手势示意</p>
      </div>
    </div>
  </div>
</template>

<script>
export default {
  data() {
    return {};
  },
  mounted() {
    this.initVideo();
  },
  beforeDestroy() {
    stopCamera.callService(null, (res) => {
      console.log('[ cam_stop ok]-61', res)
    }, (res) => {
      console.log('[ cam_stop ERR]-61', res)
    });
    if (window.aidShowBridge && window.aidShowBridge.close) {
      window.aidShowBridge.close();
    }
  },
  methods: {
    initVideo() {
      const rgbWidth = this.$route.query.w;
      const rgbHeight = this.$route.query.h;
      const video = document.getElementById('video')
      const top = video.getBoundingClientRect().top
      const left = video.getBoundingClientRect().left
      const width = video.getBoundingClientRect().width
      const height = video.getBoundingClientRect().height
      if (window.aidShowBridge && window.aidShowBridge.setSurfaceLocation) {
        window.aidShowBridge.setSurfaceLocation(left, top, width, height, Number(rgbWidth), Number(rgbHeight), 1920);
      }
    },
  },
};
</script>

<style lang="less" scoped>
.gesture-container {
  box-sizing: border-box;
  display: flex;
  color: #fff;
  width: 100%;
  height: 100%;
  position: relative;
  padding-top: 30px;
  padding-left: 30px;
}

.video-container {
  position: relative;
  top: 0;
  height: 1010px;
  width: 1380px;
  //background: transparent;
  //border-radius: 5px;
  background: #526cad;
  //overflow: hidden;
}

.right {
  width: 434px;
  height: 1010px;
  background-color: #ccc;
  border-radius: 5px;
  background: linear-gradient(
    155deg,
    rgba(71, 84, 141, 0.64) 24%,
    rgba(71, 66, 124, 0.52) 98%
  );
  backdrop-filter: blur(10.88px);
  box-shadow: 0px 2px 31px 0px rgba(1, 29, 90, 0.72);
  display: flex;
  flex-direction: column;
  align-items: center;
  line-height: 50px;
  margin-left: 30px;
  padding: 20px 20px 70px 20px;
  box-sizing: border-box;

  .rText {
    min-height: 40%;
    display: flex;
    align-items: center;
    flex-direction: column;
    padding: 20px 34px;
    p {
      line-height: 45px;
      text-align: center;
    }
    .title {
      padding-bottom: 10px;
    }
    .description {
      margin-top: 35px;
    }
  }
  .rLogo {
    img {
      margin-top: 20px;
    }
    p {
      margin-top: 10px;
    }
  }
}
</style>
