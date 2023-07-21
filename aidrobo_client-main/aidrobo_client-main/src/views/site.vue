<template>
  <div class="main">
    <div class="box">
      <div v-for="(item, i) in box" :key="i">
        <div class="title">{{ item.title }}</div>
        <!-- 基础设置 -->
        <div v-if="item.title === '表情锁屏'">
          <div class="item">
            <span>开启/关闭</span>
            <el-switch v-model="value" active-color="#05E29E" inactive-color="#1A1931" active-text="开" inactive-text="关">
            </el-switch>
          </div>
        </div>
        <div v-else>
          <div v-for="(v,k) in item.info" :key="k" class="item">{{ k }}:{{ v}}</div>
        </div>
      </div>
    </div>
  </div>
</template>

<script>
export default {
  data () {
    return {
      value: true,
      input: '',
      box: [
        // {
        //   title: "表情锁屏"
        // },
        // {
        //   title: "设备信息",
        //   info: {
        //     '机器编号(SN)': "AIDR-T2301110001",
        //     '软件版本': "0.5.5.1.d",
        //   }
        // },
        {
          title: "主要硬件信息",
          info: {
            '主板': "Firfly AIO-3588Q  8+64G",
            '主控': "艺科 YKRC-2",
            '激光雷达': "蓝海 LDS-50C-C20E",
            '双目模组': "奥比中光 DaBai",
          },

        },

      ]
    }
  }
}
</script>

<style lang="less" scoped>
.main {
  overflow: auto;
  height: 100%;
}

.box {
  display: flex;
  flex-wrap: wrap;
  color: #fff;

  &>div {
    padding: 40px 70px;
    width: 670px;
    min-height: 220px;
    border-radius: 20px;
    background: linear-gradient(116deg, rgba(71, 84, 141, 0.64) 12%, rgba(71, 66, 124, 0.52) 90%);
    backdrop-filter: blur(10.88px);
    box-shadow: 0px 2px 31px 0px rgba(1, 29, 90, 0.72);
    margin-top: 30px;
    margin-bottom: 30px;
    margin-left: 100px;

    .title {
      font-size: 40px;
      font-weight: bold;
    }

    &>div {
      margin-top: 40px;

      .item {
        margin-top: 30px;
        display: flex;
        align-items: center;
        justify-content: space-between;
        width: 600px;

        .el-switch {
          transform: scale(2.5);
          margin-right: 30px;
        }
      }

      .itemTip {
        font-size: 28px;
        color: #C1C1C1;
      }
    }
  }
}
</style>
<style  scoped>
.item>>>.el-switch__label {
  position: absolute;
  display: none;
  color: #fff;
}

/*打开时文字位置设置*/
.item>>>.el-switch__label--right {
  z-index: 1;
  right: 18px;
}

/*关闭时文字位置设置*/
.item>>>.el-switch__label--left {
  z-index: 1;
  left: 18px;
}

/*显示文字*/
.item>>>.el-switch__label.is-active {
  display: block;
}

.timeInp>>>.el-input {
  width: 140px;
  font-size: 35px;
}

.filesInp>>>.el-input {
  width: 300px;
  font-size: 35px;
}

.item>>>.el-input__inner {
  height: 70px;
  border-radius: 10px;

}
</style>
