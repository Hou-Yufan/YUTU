const ros = new ROSLIB.Ros();
ros.connect('ws://192.168.1.120:9090');
// ros.connect('ws://192.168.110.204:9090');
/** 模式切换 */
const robotMode = new ROSLIB.Service({
  ros: ros,
  name: '/mode_set',
  serviceType: 'aid_robot_msgs/srv/StatusChange'
});

/** 建图订阅 */
const robotMap = new ROSLIB.Topic({
  ros: ros,
  name: '/map_base64',
  messageType: 'nav_msgs/msg/OccupancyGrid'
});
/** 建图结束 */
const finishMap = new ROSLIB.Service({
  ros: ros,
  name: '/finish_trajectory',
  serviceType: 'cartographer_ros_msgs/srv/FinishTrajectory'
});
/** 保存建图文件 */
const saveMapFile = new ROSLIB.Service({
  ros: ros,
  name: '/write_state',
  serviceType: 'cartographer_ros_msgs/srv/WriteState'
});
/** 建图集成保存 */
const saveMap = new ROSLIB.Service({
  ros: ros,
  name: '/aid_save_map',
  serviceType: 'aid_robot_msgs/srv/MapOperation'
});
/** 保存建图db */
const saveMapDb = new ROSLIB.Service({
  ros: ros,
  name: '/add_map',
  serviceType: 'aid_robot_msgs/srv/MapOperationAdd'
});
/** 获取地图列表 */
const getMapList = new ROSLIB.Service({
  ros: ros,
  name: '/get_map_list',
  serviceType: 'aid_robot_msgs/srv/MapList'
});
/** 删除地图 */
const deleteMap = new ROSLIB.Service({
  ros: ros,
  name: '/delete_map',
  serviceType: 'aid_robot_msgs/srv/OperationDelete'
});
/** 更新地图 */
const updateMap = new ROSLIB.Service({
  ros: ros,
  name: '/update_map',
  serviceType: 'aid_robot_msgs/srv/OperationUpdate'
});
/** 获取地图数据 */
const getMapImage = new ROSLIB.Service({
  ros: ros,
  name: '/get_map_image',
  serviceType: 'aid_robot_msgs/srv/MapImage'
});
/** 设置当前地图为使用中的地图 */
const setCurrentMapId = new ROSLIB.Service({
  ros: ros,
  name: '/set_current_map_id',
  serviceType: 'aid_robot_msgs/srv/SetCurrentMap'
});
/** 获取当前使用中的地图id */
const getCurrentMapId = new ROSLIB.Service({
  ros: ros,
  name: '/get_current_map_id',
  serviceType: 'aid_robot_msgs/srv/GetCurrentMap'
});
/** 机器人在地图中位置*/
const robotPosition = new ROSLIB.Topic({
  ros: ros,
  name: '/base_link_pose',
  messageType: 'geometry_msgs/msg/PoseStamped'
});
/** 重定位*/
const PoseStamped = new ROSLIB.Topic({
  ros: ros,
  name: '/goal_pose',
  messageType: 'geometry_msgs/msg/PoseStamped'
});
/** 获取巡逻点列表 */
const getMapLinkedDataList = new ROSLIB.Service({
  ros: ros,
  name: '/get_map_waypoint_list',
  serviceType: 'aid_robot_msgs/srv/MapLinkedDataList'
});
/** 新增巡逻 */
const OperationAdd = new ROSLIB.Service({
  ros: ros,
  name: '/add_waypoint',
  serviceType: 'aid_robot_msgs/srv/OperationAdd'
});
/** 更新巡逻点 */
const OperationUpdate = new ROSLIB.Service({
  ros: ros,
  name: '/update_waypoint',
  serviceType: 'aid_robot_msgs/srv/OperationUpdate'
});
/** 获取禁行线 */
const ForbiddenGet = new ROSLIB.Service({
  ros: ros,
  name: '/get_forbidden',
  serviceType: 'aid_robot_msgs/srv/ForbiddenGet'
});
/** 新增禁行线 */
const ForbiddenAdd = new ROSLIB.Service({
  ros: ros,
  name: '/add_forbidden',
  serviceType: 'aid_robot_msgs/srv/OperationAdd'
});
/** 更新禁行线 */
const ForbiddenUpdate = new ROSLIB.Service({
  ros: ros,
  name: '/update_forbidden',
  serviceType: 'aid_robot_msgs/srv/OperationUpdate'
});
/** 东方禁行线 */
const DrawPicture = new ROSLIB.Service({
  ros: ros,
  name: '/aid_draw_forbidden_line',
  serviceType: 'aid_robot_msgs/srv/DrawPicture'
});

/** 发送位置点*/
const TalkerPoint = new ROSLIB.Topic({
  ros: ros,
  name: '/patrol_path',
  messageType: 'nav_msgs/msg/Path'
});
/** 停止 */
const stopPatrol = new ROSLIB.Topic({
  ros: ros,
  name: '/stop_patrol',
  messageType: 'std_msgs/Empty'
});
/** 运动状态控制 */
const patrolState = new ROSLIB.Service({
  ros: ros,
  name: '/patrol_control',
  serviceType: 'aid_robot_msgs/srv/PatrolControl'
});




/** 页面初始化摄像头 */
const startCamera = new ROSLIB.Service({
  ros: ros,
  name: '/cam_start',
  serviceType: 'aid_robot_msgs/srv/AICmd'
});
/** 页面停止摄像头 */
const stopCamera = new ROSLIB.Service({
  ros: ros,
  name: '/cam_stop',
  serviceType: 'aid_robot_msgs/srv/AICmd'
});
/** 获取人物识别状态 */
const getFollowStatus = new ROSLIB.Topic({
  ros: ros,
  name: '/follow_status',
  messageType: 'std_msgs/msg/Bool'
})
/** 特征跟随-开始跟随 */
const startFollow = new ROSLIB.Service({
  ros: ros,
  name: '/follow_start',
  serviceType: 'aid_robot_msgs/srv/AICmd'
});
/** 特征跟随-停止跟随 */
const stopFollow = new ROSLIB.Service({
  ros: ros,
  name: '/follow_stop',
  serviceType: 'aid_robot_msgs/srv/AICmd'
});
/** 获取ip */
const GetStrings = new ROSLIB.Service({
  ros: ros,
  name: '/get_ip_addresses',
  serviceType: 'aid_robot_msgs/srv/GetString'
});
/** 获取电量信息 */
const BatteryState = new ROSLIB.Topic({
  ros: ros,
  name: '/battery_data',
  serviceType: 'sensor_msgs/msg/BatteryState'
});
ros.on('connection', function () {
  console.log('rosOk!!!');
});
