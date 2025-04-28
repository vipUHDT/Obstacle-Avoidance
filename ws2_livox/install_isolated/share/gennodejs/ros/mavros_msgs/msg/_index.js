
"use strict";

let BatteryStatus = require('./BatteryStatus.js');
let HilSensor = require('./HilSensor.js');
let GPSINPUT = require('./GPSINPUT.js');
let StatusText = require('./StatusText.js');
let ESCInfoItem = require('./ESCInfoItem.js');
let GPSRAW = require('./GPSRAW.js');
let VFR_HUD = require('./VFR_HUD.js');
let OnboardComputerStatus = require('./OnboardComputerStatus.js');
let Waypoint = require('./Waypoint.js');
let ESCInfo = require('./ESCInfo.js');
let HilStateQuaternion = require('./HilStateQuaternion.js');
let TerrainReport = require('./TerrainReport.js');
let ActuatorControl = require('./ActuatorControl.js');
let TimesyncStatus = require('./TimesyncStatus.js');
let OpticalFlowRad = require('./OpticalFlowRad.js');
let MagnetometerReporter = require('./MagnetometerReporter.js');
let CompanionProcessStatus = require('./CompanionProcessStatus.js');
let FileEntry = require('./FileEntry.js');
let LandingTarget = require('./LandingTarget.js');
let Trajectory = require('./Trajectory.js');
let EstimatorStatus = require('./EstimatorStatus.js');
let HilControls = require('./HilControls.js');
let ParamValue = require('./ParamValue.js');
let ADSBVehicle = require('./ADSBVehicle.js');
let RadioStatus = require('./RadioStatus.js');
let State = require('./State.js');
let ESCStatus = require('./ESCStatus.js');
let HilGPS = require('./HilGPS.js');
let CommandCode = require('./CommandCode.js');
let DebugValue = require('./DebugValue.js');
let WheelOdomStamped = require('./WheelOdomStamped.js');
let CamIMUStamp = require('./CamIMUStamp.js');
let PlayTuneV2 = require('./PlayTuneV2.js');
let ESCStatusItem = require('./ESCStatusItem.js');
let ESCTelemetryItem = require('./ESCTelemetryItem.js');
let Thrust = require('./Thrust.js');
let RTKBaseline = require('./RTKBaseline.js');
let GPSRTK = require('./GPSRTK.js');
let RCOut = require('./RCOut.js');
let RTCM = require('./RTCM.js');
let RCIn = require('./RCIn.js');
let WaypointReached = require('./WaypointReached.js');
let LogEntry = require('./LogEntry.js');
let HilActuatorControls = require('./HilActuatorControls.js');
let NavControllerOutput = require('./NavControllerOutput.js');
let GlobalPositionTarget = require('./GlobalPositionTarget.js');
let HomePosition = require('./HomePosition.js');
let VehicleInfo = require('./VehicleInfo.js');
let PositionTarget = require('./PositionTarget.js');
let ExtendedState = require('./ExtendedState.js');
let Vibration = require('./Vibration.js');
let AttitudeTarget = require('./AttitudeTarget.js');
let SysStatus = require('./SysStatus.js');
let Param = require('./Param.js');
let MountControl = require('./MountControl.js');
let CameraImageCaptured = require('./CameraImageCaptured.js');
let CellularStatus = require('./CellularStatus.js');
let Altitude = require('./Altitude.js');
let Tunnel = require('./Tunnel.js');
let OverrideRCIn = require('./OverrideRCIn.js');
let Mavlink = require('./Mavlink.js');
let ManualControl = require('./ManualControl.js');
let WaypointList = require('./WaypointList.js');
let LogData = require('./LogData.js');
let ESCTelemetry = require('./ESCTelemetry.js');

module.exports = {
  BatteryStatus: BatteryStatus,
  HilSensor: HilSensor,
  GPSINPUT: GPSINPUT,
  StatusText: StatusText,
  ESCInfoItem: ESCInfoItem,
  GPSRAW: GPSRAW,
  VFR_HUD: VFR_HUD,
  OnboardComputerStatus: OnboardComputerStatus,
  Waypoint: Waypoint,
  ESCInfo: ESCInfo,
  HilStateQuaternion: HilStateQuaternion,
  TerrainReport: TerrainReport,
  ActuatorControl: ActuatorControl,
  TimesyncStatus: TimesyncStatus,
  OpticalFlowRad: OpticalFlowRad,
  MagnetometerReporter: MagnetometerReporter,
  CompanionProcessStatus: CompanionProcessStatus,
  FileEntry: FileEntry,
  LandingTarget: LandingTarget,
  Trajectory: Trajectory,
  EstimatorStatus: EstimatorStatus,
  HilControls: HilControls,
  ParamValue: ParamValue,
  ADSBVehicle: ADSBVehicle,
  RadioStatus: RadioStatus,
  State: State,
  ESCStatus: ESCStatus,
  HilGPS: HilGPS,
  CommandCode: CommandCode,
  DebugValue: DebugValue,
  WheelOdomStamped: WheelOdomStamped,
  CamIMUStamp: CamIMUStamp,
  PlayTuneV2: PlayTuneV2,
  ESCStatusItem: ESCStatusItem,
  ESCTelemetryItem: ESCTelemetryItem,
  Thrust: Thrust,
  RTKBaseline: RTKBaseline,
  GPSRTK: GPSRTK,
  RCOut: RCOut,
  RTCM: RTCM,
  RCIn: RCIn,
  WaypointReached: WaypointReached,
  LogEntry: LogEntry,
  HilActuatorControls: HilActuatorControls,
  NavControllerOutput: NavControllerOutput,
  GlobalPositionTarget: GlobalPositionTarget,
  HomePosition: HomePosition,
  VehicleInfo: VehicleInfo,
  PositionTarget: PositionTarget,
  ExtendedState: ExtendedState,
  Vibration: Vibration,
  AttitudeTarget: AttitudeTarget,
  SysStatus: SysStatus,
  Param: Param,
  MountControl: MountControl,
  CameraImageCaptured: CameraImageCaptured,
  CellularStatus: CellularStatus,
  Altitude: Altitude,
  Tunnel: Tunnel,
  OverrideRCIn: OverrideRCIn,
  Mavlink: Mavlink,
  ManualControl: ManualControl,
  WaypointList: WaypointList,
  LogData: LogData,
  ESCTelemetry: ESCTelemetry,
};
