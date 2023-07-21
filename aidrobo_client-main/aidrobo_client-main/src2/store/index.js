import Vue from 'vue'
import Vuex from 'vuex'
Vue.use(Vuex)
const store = new Vuex.Store({
  state: {
    mcode: '', //操作选择
    rubber_data1: false, //不规则区域
    rubber_data2: false, //选择橡皮檫
    rubber_size: 20, //橡皮檫大小
    stop_point: 0, //禁行区计数点
    head_h: 150, //工具区域
    zero: 0, //不规则区域点计数

    tool: '',//选取的工具
    x_can: null, //canvas
    _map_name: '', //地图名字
    map_width: null,//发送给后台将获取的地图宽
    m_width: 992,//地图IMG宽
    m_height: 992,//地图IMG高
    m_resolution: 0.05, //地图分辨率
    m_position:{},//地图原点
    m_resolution2:0,//地图物理分辨率
    charge_po_back:null,//后台传来充电桩坐标
    set_state:null,//地图操作成功与否
    init_img_data: true,//加载地图是否初始化
    right_top_save:false,//是否为左上保存
    save_map:false,//保存地图
    txt:'等待中...',
    loading_build: false, //构建地图等待
    loading_dev:false,//编辑地图等待
    tool_active:0,//工具栏选项
    prepro_val:0,//孤立点大小值
    charge_po:[],//充电桩位置
    robotPoint:{x:0,y:0},
    mapSrc: '',
    help: false,//帮助栏开关
    map_img_w:0,//屏幕地图宽
    set_msg:null,//接收錯誤消息
    patrol_chang_data:[],
    //
    actionStatus:'',//前端操作状态point/patrolStart(巡逻中)/patrolPause(暂停巡逻)/remote_control/localization/newMap/
    linearCurveArr:[],//禁行线点位
    linearCurveArrP:[],//禁行线点位
    eraserArr:[],//橡皮擦点位
    mapData:{
      src: "",
      width: 1930,
      height: 3909,
      resolution:0.05000000074505806,
      positionX: 0,
      positionY: 0,
    },//地图数据
    showMsg:false,//控制msg提示框显示
    hasSave:true,//顶部路由=》页面保存提示控制
    nowMap:{id:"",name:''},
    patrol_arr_area:[],//图像坐标巡逻点
    patrol_arr: [],//地图坐标巡逻点
  },
  getters: {
    mcode: state => state.mcode,
  },
  mutations: {
    chang_data(state, n) {
      state.mcode = n;
    },
    rubber_chang_data1(state, e) {//构建修改值得方法
      e != undefined ? state.rubber_data1 = e : state.rubber_data1 = !state.rubber_data1;
    }
  }
})
export default store
