<template>
  <div class="headBox">
    <div class="toolRow">
      <div v-if="routerN !== '/'">
        <div class="home" @click="routerFun('/')">
          <img src="@/assets/img/homeico.svg" />
        </div>
        <div class="home home2" @click="routerFun()">
          <img src="@/assets/img/back.svg" />
        </div>
        <span class="routerTxt">{{ routerTxt }}</span>
      </div>
      <div v-else>
        <div class="home" @click="refreshFun()">
          <img src="@/assets/img/home/refresh.png" />
        </div>
      </div>
      <div class="rowR">
        <div class="show" @touchstart="routerStart()" @touchend="routerEnd()">
          <img src="@/assets/img/home/patrol.png" />
          <span>sdfsdfsdfsdfsdfdsf{{ cmd }}</span>
        </div>
        <div class="show" @click="relocation()">
          <img src="@/assets/img/home/mapName.png" />
          <span>sdfsdfsdfsdfsdfdsf{{ this.$store.state.nowMap.name || '无地图' }}</span>
        </div>
        <div class="electric">
          <img src="@/assets/img/home/electric.png" />
          <div>
            <div :style="{ 'width': '100%' }"></div>
          </div>
          {{ electric }}%
        </div>
      </div>
    </div>
  </div>
</template>

<script>
import { mapState, mapMutations } from "vuex";
import { routerObj } from '@/assets/common'
export default {
  computed: {
    ...mapState([
      "showMsg",
      "hasSave",
      "actionStatus"
    ])
  },
  data () {
    return {
      electric: 100,
      showTc: false,
      set: null,
      luminance: 20,
      cmd: '',
      routerN: '',
      routerTxt: '',
      voice: 0
    };
  },
  watch: {
    actionStatus: function (n) {
      if (n === 'patrolStart') {
        this.cmd = '巡逻中...';
      } else if (n === 'patrolPause') {
        this.cmd = '恢复巡逻';
      }
    },
    $route (to, from) {
      this.routerN = to.path
      this.routerTxt = routerObj[to.name]
      // console.log('//从哪来',from.path);
      // console.log('//到哪去', to);
    }
  },
  methods: {
    refreshFun () {
      window.location.reload();
    },
    relocation () {
      console.log('头部点击定位')
      const modeMsg = new ROSLIB.ServiceRequest({
        action: 'localization'
      });
      robotMode.callService(modeMsg, (result) => {
        console.log('[ robotMode OK]-61', result)
        this.$message('定位成功');
      }, (result) => {
        console.log('[ robotMode ERR]-61', result)
        this.$message('定位失败');
      });
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
    },
    patrolAction () {
      if (this.actionStatus === 'patrolStart') {
        const type = new ROSLIB.ServiceRequest({
          cmd: 'pause'
        });
        patrolState.callService(type, (res) => {
          console.log('[ patrol_control ok]-61', res)
        }, (res) => {
          console.log('[ patrol_control ERR]-61', res)
        });
        this.$store.state.actionStatus = 'patrolPause'
      } else {
        this.$store.state.actionStatus = 'patrolStart'
        const type = new ROSLIB.ServiceRequest({
          cmd: 'resume'
        });
        patrolState.callService(type, (res) => {
          console.log('[ patrol_control ok]-61', res)
        }, (res) => {
          console.log('[ patrol_control ERR]-61', res)
        });
      }
    },
    routerStart () {
      this.patrolAction()
      let num = 0;
      this.set = setInterval(() => {
        num += 1
        if (num >= 3) {
          if (this.actionStatus.includes('patrol')) {
            this.$router.push('/utility/patrol')
          }
          clearInterval(this.set)
        }
      }, 1000)
    },
    routerEnd () {
      clearInterval(this.set)
    },
    routerFun (path) {
      if (this.$router.history.current.path === '/utility/goPoint') {
        if (!this.hasSave) {
          this.$confirm(`<div>是否确认退出</div><div>（已操作内容不会保存）</div>`, '退出', {
            dangerouslyUseHTMLString: true,
            center: true
          }).then(() => {
            if (path === '/') {
              this.$router.push('/')
            } else {
              this.$router.go(-1)
            }
          }).catch(() => {
          });
        }
      } else {
        if (path === '/') {
          this.$router.push('/')
        } else {
          this.$router.go(-1)
        }
      }
    },
    add (type, fun) {
      this.$store.state.showMsg = true
      this[type] < 100 && (this[type] += 1);
    },
    minus (type, fun) {
      this[type] > 0 && (this[type] -= 1);
    }
  },
  mounted () {
    console.log("[  ]-45", this.$router.history.current.path);
    this.routerN = this.$router.history.current.path
  }
};
</script>

<style lang="less" scoped>
.headBox {
  position: absolute;
  left: 0px;
  top: 0px;
  width: 1920px;
  height: 120px;
  opacity: 1;
  z-index: 10;
  color: #fff;
  background: #495BAF;
  backdrop-filter: blur(10.88px);
  box-shadow: 0px 2px 31px 0px rgba(1, 29, 90, 0.72);
}

.toolRow {
  width: 100%;
  height: 100%;

  .home {
    width: 80px;
    height: 80px;
    background: #2F3758;
    border-radius: 10px;
    margin: 20px;
    position: absolute;
    display: flex;
    align-items: center;
    justify-content: center;
  }

  .home2 {
    position: absolute;
    left: 100px;
  }

  .rowR {
    width: 57%;
    height: 100%;
    margin-left: 42%;
    position: relative;
    display: flex;
    align-items: center;
    justify-content: space-between;

    &>div {
      height: 80px;
    }

    .show {
      color: #fff;
      background: #000;
      width: 350px;
      border-radius: 10px;
      opacity: 1;
      background: #2f3758;
      backdrop-filter: blur(10px);
      display: flex;
      align-items: center;
      padding: 0 30px;
      display: flex;
      align-items: center;
      justify-content: center;
      &>span {
        margin-left: 20px;
        overflow: hidden;
        text-overflow: ellipsis;
        flex-shrink: 0;
        max-width: 200px;
        height: 100%;
        line-height: 80px;
      }
    }

    .electric {
      position: relative;
      display: flex;
      align-items: center;
      width: 177px;

      &>img {
        margin-right: 10px;
      }

      &>div {
        right: 98px;
        width: 72px;
        position: absolute;

        &>div {
          height: 25px;
          background: #c6cfe9;
        }
      }
    }

    .Tool {
      width: 80px;
      background: #33478b;
      border-radius: 50%;
    }
  }
}

/*  */
.toolBox {
  position: absolute;
  right: 60px;
  top: 125px;
  width: 634px;
  height: 534.28px;
  opacity: 1;
  background: #374275;
  box-sizing: border-box;
  border: 2px solid rgba(162, 155, 229, 0.32);
  backdrop-filter: blur(10px);
  box-shadow: 0px 0px 5px 0px rgba(0, 0, 0, 0.23);

  .btn {
    position: absolute;
    width: 338.35px;
    height: 87.54px;
    border-radius: 10px;
    opacity: 1;
    background: #232850;
    box-shadow: 0px 0px 20px 0px rgba(0, 0, 0, 0.3);
  }

  .btn1 {
    left: 52.56px;
    top: 87.82px;
  }

  .btn2 {
    left: 52.56px;
    top: 202.71px;
  }

  .btn3 {
    left: 53px;
    top: 404.06px;
  }

  .btn3::before {
    content: "";
    width: 100%;
    height: 1px;
    background: #e1dddd;
    position: absolute;
    top: -42px;
  }

  .rightBox {
    position: absolute;
    left: 447.85px;
    top: 85px;
    width: 148.97px;
    height: 346.64px;
    opacity: 1;
    display: flex;
    justify-content: space-between;

    .rTop {
      width: 53.83px;
      height: 76.89px;
      background: #232850;
    }

    .rBody {
      width: 53.83px;
      height: 193.11px;
      background: #bfc9f9;
      position: relative;

      &>div {
        position: absolute;
        width: 53.83px;
        bottom: 0;
        background: #ccc;
      }
    }
  }
}

.routerTxt {
  position: absolute;
  left: 230px;
  top: 46px;
}
</style>
