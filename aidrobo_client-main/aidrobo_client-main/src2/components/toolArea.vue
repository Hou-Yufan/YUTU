<template>
  <div class="tool" ref="tool">
    <div class="tool_box">
      <div
        v-for="(item, index) in items"
        :key="index"
        @click="jumpTo(index)"
        class="btn"
      >
        <img :src="item.cla" width="34px" />
        <span>{{ item.name }}</span>
      </div>
    </div>
    <router-view></router-view>
    <!-- <div class="bottom">
     <img src="../../static2/img/logo.png" width="80%"/>
    </div> -->
  </div>
</template>
<script type="text/ecmascript-6">
import { mapState, mapMutations } from "vuex";
import dw from  "../../static2/img/btn/dw.png";
import xl from  "../../static2/img/btn/xl.png";
import yk from  "../../static2/img/btn/yk.png";
import cdz from  "../../static2/img/btn/cdz.png";
export default {
  data() {
    return {
      items: [
        {
          name: "去位置点",
          cla: dw
        },
        {
          name: "巡逻模式",
          cla: xl
        },
        {
          name: "遥控模式",
          cla: yk
        },
        {
          name: "回充返回",
          cla: cdz
        }
      ],
      show: false,
      lastStep: false,
      nextStep: false,
      save: false,
      go_next: false,
      active: 0
    };
  },
  computed: {
    ...mapState([
      "mcode",
      "rubber_data",
      "head_h",
      "go_rubber",
      "zero",
      "x_can",
      "tool_active",
      "_map_name",
      "stop_chang_data",
      "stop_point"
    ])
  },
  mounted() {
    console.log("[ innerWidth ]-68", window.innerWidth);
    console.log("[ innerHeight ]-70", window.innerHeight);
    this.chang_data(0);
  },
  methods: {
    ...mapMutations(["chang_data", "rubber_chang_data1"]),
    jumpTo(e) {
      this.$store.state.init_img_data = false; //加载地图否初始化
      this.chang_data(e);
      this.$store.state.tool = "";
      switch (e) {
        case 0:
          this.$router.push({
            path: "/point"
          });
          break;
        case 1:
          this.$router.push({
            path: "/patrol"
          });
          break;
        case 2:
          this.$router.push({
            path: "/control"
          });
          break;
        case 3:
          this.$router.push({
            path: "/charge"
          });
          break;
      }
    }
  },
  components: {}
};
</script>

<style>
.tool {
  position: absolute;
  top: 0;
  left: 75%;
  height: 100%;
  width: 25%;
}

.tool_box {
  height: 80%;
  width: 100%;
  display: flex;
  flex-flow: column;
  align-items: center;
}

.btn {
  display:flex;
  align-items: center;
  justify-content: center;
  color: #fff;
  width: 80%;
  height: 4rem;
  font-size: 1rem;
  background: #f3c8aa;
  color: #fff;
  margin-top: 2rem;
  border-radius: 4px;
  background: linear-gradient(
    101deg,
    rgba(71, 84, 141, 0.64) 9%,
    rgba(53, 81, 119, 0.15) 87%,
    rgba(53, 92, 119, 0.14) 88%
  );
  backdrop-filter: blur(10.88px);
  box-shadow: 0px 2px 8px 0px rgba(1, 29, 90, 0.72);
}
.btn_tr {
  background: #0cd4af;
  color: #fff;
}

.rightbtn {
  position: absolute;
  right: 0;
  top: 0;
  width: 20%;
  height: 100%;
  display: flex;
  flex-direction: column;
  justify-content: center;
  align-items: center;
}

.right_box {
  position: absolute;
  width: 100%;
  height: 90%;
  border-left: 1px solid #0cd5af;
  display: flex;
  flex-direction: column;
  justify-content: center;
  align-items: center;
}

.save {
  float: left;
  width: 170px;
  height: 50px;
  margin-top: 20px;
  background: #d9dbdb;
  color: #525f5f;
  font-size: 24px;
  border-radius: 11px;
  display: flex;
  justify-content: center;
  align-items: center;
  box-shadow: 1px 1px 11px #0cd5af;
}

.saveend {
  background: #0cd4af;
  box-shadow: none;
  color: #fff;
}

.repeal {
  float: left;
  width: 170px;
  height: 50px;
  display: flex;
  justify-content: flex-start;
  align-items: center;
}

.repeal_touchend {
  width: 40%;
  height: 100%;
  background: #d9dbdb;
  border: 1px solid #d9dbdb;
  box-shadow: 1px 1px 11px #0cd5af;
  float: left;
  display: flex;
  justify-content: center;
  align-items: center;
  border-radius: 11px;
  color: #525f5f;
  font-size: 36px;
}

.repeal_touchst {
  box-shadow: none;
  background: #0cd4af;
  border: 1px solid #d9dbdb;
  color: #d9dbdb;
}

.repeal div:last-child {
  margin-left: 30px;
}
.bottom {
  width: 100%;
  position: absolute;
  bottom: 1rem;
  display: flex;
  align-items: center;
  justify-content: center;
  font-size: 1.25rem;
}
</style>
