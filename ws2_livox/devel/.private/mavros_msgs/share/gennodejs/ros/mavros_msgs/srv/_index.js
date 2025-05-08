
"use strict";

let FileRemove = require('./FileRemove.js')
let CommandVtolTransition = require('./CommandVtolTransition.js')
let WaypointPush = require('./WaypointPush.js')
let VehicleInfoGet = require('./VehicleInfoGet.js')
let ParamPull = require('./ParamPull.js')
let FileMakeDir = require('./FileMakeDir.js')
let FileRemoveDir = require('./FileRemoveDir.js')
let CommandHome = require('./CommandHome.js')
let ParamSet = require('./ParamSet.js')
let FileRename = require('./FileRename.js')
let WaypointPull = require('./WaypointPull.js')
let FileRead = require('./FileRead.js')
let StreamRate = require('./StreamRate.js')
let CommandTriggerInterval = require('./CommandTriggerInterval.js')
let FileChecksum = require('./FileChecksum.js')
let MountConfigure = require('./MountConfigure.js')
let ParamGet = require('./ParamGet.js')
let CommandAck = require('./CommandAck.js')
let SetMode = require('./SetMode.js')
let FileOpen = require('./FileOpen.js')
let LogRequestList = require('./LogRequestList.js')
let LogRequestData = require('./LogRequestData.js')
let FileWrite = require('./FileWrite.js')
let FileList = require('./FileList.js')
let CommandLong = require('./CommandLong.js')
let SetMavFrame = require('./SetMavFrame.js')
let FileTruncate = require('./FileTruncate.js')
let ParamPush = require('./ParamPush.js')
let CommandTriggerControl = require('./CommandTriggerControl.js')
let WaypointSetCurrent = require('./WaypointSetCurrent.js')
let CommandInt = require('./CommandInt.js')
let FileClose = require('./FileClose.js')
let CommandTOL = require('./CommandTOL.js')
let CommandBool = require('./CommandBool.js')
let LogRequestEnd = require('./LogRequestEnd.js')
let MessageInterval = require('./MessageInterval.js')
let WaypointClear = require('./WaypointClear.js')

module.exports = {
  FileRemove: FileRemove,
  CommandVtolTransition: CommandVtolTransition,
  WaypointPush: WaypointPush,
  VehicleInfoGet: VehicleInfoGet,
  ParamPull: ParamPull,
  FileMakeDir: FileMakeDir,
  FileRemoveDir: FileRemoveDir,
  CommandHome: CommandHome,
  ParamSet: ParamSet,
  FileRename: FileRename,
  WaypointPull: WaypointPull,
  FileRead: FileRead,
  StreamRate: StreamRate,
  CommandTriggerInterval: CommandTriggerInterval,
  FileChecksum: FileChecksum,
  MountConfigure: MountConfigure,
  ParamGet: ParamGet,
  CommandAck: CommandAck,
  SetMode: SetMode,
  FileOpen: FileOpen,
  LogRequestList: LogRequestList,
  LogRequestData: LogRequestData,
  FileWrite: FileWrite,
  FileList: FileList,
  CommandLong: CommandLong,
  SetMavFrame: SetMavFrame,
  FileTruncate: FileTruncate,
  ParamPush: ParamPush,
  CommandTriggerControl: CommandTriggerControl,
  WaypointSetCurrent: WaypointSetCurrent,
  CommandInt: CommandInt,
  FileClose: FileClose,
  CommandTOL: CommandTOL,
  CommandBool: CommandBool,
  LogRequestEnd: LogRequestEnd,
  MessageInterval: MessageInterval,
  WaypointClear: WaypointClear,
};
