<template>
  <div class="main">
    <el-carousel
      indicator-position="none"
      arrow="never"
      :autoplay="false"
      ref="carousel"
      class="box"
      height="930px"
    >
      <el-carousel-item
        v-for="(page, pageIndex) in pages"
        :key="`page-${pageIndex}`"
      >
        <div class="page-container">
          <div @click="goPage(item)" class="link-item" v-for="(item, i) in page" :key="i">
            <router-link to="" class="link">
              <img :src="item.src" />
              <p>{{ item.text }}</p>
            </router-link>
          </div>
        </div>
        <router-view />
      </el-carousel-item>

      <!-- 翻页键 -->
      <div v-if="pages.length > 1">
        <div
          :class="['left', { disabled: pageIndex === 0 }]"
          @click="changePage('left', pageIndex === 0)"
        >
          <img src="@/assets/img/back.svg" />
        </div>
        <div
          :class="['right', { disabled: pageIndex === pages.length - 1 }]"
          @click="changePage('right', pageIndex === pages.length - 1)"
        >
          <img src="@/assets/img/back.svg" />
        </div>
      </div>
    </el-carousel>
  </div>
</template>

<script>
import fullscreenLoading from "../../components/fullscreenLoading.js";

export default {
  data () {
    return {
      // imgSrc 中，需要执行 startCamera 的才加 cmd 字段
      imgSrc: [
        // { src: require("@/assets/img/uti/uti1.svg"), text: '重定位', link: "/utility/relocation" },
        { src: require("@/assets/img/uti/uti2.svg"), text: '去位置点', link: "/utility/goPoint" },
        { src: require("@/assets/img/uti/uti3.svg"), text: '自主巡逻', link: "/utility/patrol" },
        // { src: require("@/assets/img/uti/uti4.svg"), text: '返回原点/充电', link: "/utility/charge" },
        { src: require("@/assets/img/uti/uti5.svg"), text: '遥控模式', link: "/utility/telecontrol" },
        { src: require("@/assets/img/uti/uti6.svg"), text: '人形跟随', link: "/utility/following", cmd: 'follow' },
        { src: require("@/assets/img/uti/face.svg"), text: '人脸跟随', link: "/utility/following", cmd: 'face' },
        { src: require("@/assets/img/uti/uti9.svg"), text: "手势识别", link: "/utility/gesture", cmd: 'hand' },
        { src: require("@/assets/img/uti/uti7.svg"), text: '物品识别', link: "/utility/object-recognition", cmd: 'object'},
        // { src: require("@/assets/img/uti/uti8.svg"), text: '资源看板' },
      ],
      data: null,
      isSel: null,
      pageIndex: 0,
    };
  },
  computed: {
    pages() {
      let limit = 8; // 一页的条数
      let res = [];
      this.imgSrc.forEach((item, i) => {
        let page = Math.floor(i / limit);
        if (!res[page]) {
          res[page] = [];
        }
        res[page].push(item);
      });
      return res;
    },
  },
  methods: {
    changePage(direction, disabled) {
      if (disabled) return;
      switch (direction) {
        case "left":
          this.pageIndex--;
          break;
        case "right":
          this.pageIndex++;
          break;
      }
      this.$refs.carousel.setActiveItem(this.pageIndex);
    },
    async goPage(item) {
      let loading;
      try {
        let link = undefined;
        // 进入某些页面前，先运行一些特定逻辑
        if(item.cmd) {
          loading = fullscreenLoading();
          const rgbSize = await this.startCamera(item.cmd);
          if(!rgbSize.success || !rgbSize.message) {
            throw '';
          }
          const {w, h} = JSON.parse(rgbSize.message);
          loading.close();
          link = `${item.link}?w=${w}&h=${h}`
        }

        this.$router.push(link || item.link || '#');
      } catch (e) {
        e = e || 'Failed Error!';
        this.$message.error(e.message || e);
        loading && loading.close();
      }

    },
    async startCamera(cmd) {
      return new Promise((resolve, reject) => {
        const params = new ROSLIB.ServiceRequest({cmd});
        startCamera.callService(params, (res) => {
          console.log('[ cam_start ok]-61', res)
          resolve(res)
        }, (res) => {
          console.log('[ cam_start ERR]-61', res)
          reject(res)
        });
      })
    }
  },
};
</script>

<style lang="less" scoped>
.box {
  color: #fff;
  position: relative;
  .page-container {
    display: flex;
    flex-wrap: wrap;
    margin: 150px 0 0 0;
    height: 744px;
    align-content: space-between;
  }
  & .link-item {
    width: 330px;
    height: 330px;
    border-radius: 20px;
    background: linear-gradient(132deg, rgba(71, 84, 141, 0.64) 17%, rgba(53, 81, 119, 0.15) 92%, rgba(53, 92, 119, 0.14) 93%);
    backdrop-filter: blur(10.88px);
    box-shadow: 0px 2px 31px 0px rgba(1, 29, 90, 0.72);
    margin-left: 120px;

    .link {
      display: flex;
      flex-direction: column;
      align-items: center;
      justify-content: center;

      & img {
        width: 300px;
      }

      & p {
        position: absolute;
        bottom: 40px;
        text-align: center;
        width: 300px;
        color: #fff;
      }
    }
  }
  .left,
  .right {
    position: absolute;
    z-index: 3;
    top: 480px;
    width: 80px;
    height: 80px;
    border-radius: 50%;
    opacity: 1;
    background: #b04cf3;
    backdrop-filter: blur(10px);
    &.disabled {
      background: #7284c8;
    }
    img {
      position: absolute;
      top: 50%;
      left: 50%;
      transform: translate(-70%, -50%);
      color: red;
    }
  }
  .left {
    left: 40px;
  }
  .right {
    right: 40px;
    transform: rotate(180deg);
  }
}
</style>
