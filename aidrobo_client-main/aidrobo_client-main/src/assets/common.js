export const changeStr = function (res) {
  const { data, info: { width, height, resolution, origin: { position: { x, y } } } } = res
  const uint = (data) => {
    const uint8Array = new Uint8Array(data);
    const decoder = new TextDecoder('utf-8');
    return decoder.decode(uint8Array);
  }
  return {
    src: 'data:image/jpg;base64,'+uint(data),
    width,
    height,
    resolution,
    positionX: x,
    positionY: y,
  }
}
export const mapToImg = ({ mapData, y, x }) => {
  if (y) {
    return mapData.height - (y - mapData.positionY) / mapData.resolution;
  }
  if (x) {
    return (x - mapData.positionX) / mapData.resolution;
  }
}
export const imgToMap = ({ mapData, y, x }) => {
  if (y) {
    return(mapData.height - y) * mapData.resolution + mapData.positionY;
  }
  if (x) {
    return  x * mapData.resolution +  mapData.positionX;
  }
}
export const routerObj={
  'home':'首页',
  'map':'地图管理',
  'newMap':"新建地图",
  'editMap':"编辑地图",
  'seeMap':"查看地图",
  'site':"设置",
  'utility':"应用功能",
  'goPoint':"去位置点",
  'telecontrol':"遥控",
}
