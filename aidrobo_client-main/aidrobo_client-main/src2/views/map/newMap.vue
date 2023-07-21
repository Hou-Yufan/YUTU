<template>
  <div class="newMapBox">
    <ShowMap />
    <div class="right">
      <p>遥控模式已打开请使用手App遥控机器人行走建图，完成扫描后点击完成扫描进入下一步</p>
      <p>注意：起始位置为起始点或充电桩，建图需完成回环后回到该位置</p>
      <div class="over" @click="onOver()">完成扫描</div>
      <div class="out" @click="onOut()">退出</div>
    </div>
  </div>
</template>

<script>
import ShowMap from "@/components/map/new";

export default {
  components: {
    ShowMap
  },
  data () {
    return {
      mapName: this.$route.query.mapName
    }
  },
  mounted () {
  },
  methods: {
    onOver () {
      this.$confirm(`<div>是否确认完成扫描，确认后将生成地图进入编辑</div><div>（无法返回）</div>`, '完成扫描', {
        dangerouslyUseHTMLString: true,
        center: true
      }).then(() => {
        const date = Date.now()
        const msg = new ROSLIB.ServiceRequest({
          map_file_name: `/root/maps/${date}`
        });
        saveMap.callService(msg, (result) => {
          if (result.success) {
            const msg2 = new ROSLIB.ServiceRequest(
              {
                map_name: this.mapName,
                map_file: `/root/maps/${date}`
              }
            );
            saveMapDb.callService(msg2, (result) => {
              console.log('[  saveMapDb OK]-61', result)
            }, (result) => {
              console.log('[  saveMapDb ERR]-61', result)
            });
            this.$router.push({ name: 'map' })
          } else {
            this.$message('保存失败');
          }
        }, (result) => {
          console.log('[  saveMap ERR]-61', result)
        });



      })
    },
    onOut () {
      this.$confirm(`<div>是否确认退出</div><div>（已扫描地图不会保存）</div>`, '退出扫描', {
        dangerouslyUseHTMLString: true,
        center: true
      }).then(() => {
        this.$router.push({ name: 'map' })
        console.log('[  ]-72',)
    this.$store.state.actionStatus='idle'
        const msg = new ROSLIB.ServiceRequest({
          action: 'idle'
        });
        robotMode.callService(msg, (result) => {
          console.log('[  finishMap OK]-61', result)
        }, (result) => {
          console.log('[  finishMap ERR]-61', result)
        });
        console.log('[  ]-69',)
      })
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
  align-items: center;
  justify-content: space-evenly;
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

  &>p {
    text-align: center;
    text-align: center;
    line-height: 50px;
    padding: 0px 40px;
  }
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
