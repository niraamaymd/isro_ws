
"use strict";

let PositionTarget = require('./PositionTarget.js');
let ESCInfo = require('./ESCInfo.js');
let GPSRAW = require('./GPSRAW.js');
let TimesyncStatus = require('./TimesyncStatus.js');
let OnboardComputerStatus = require('./OnboardComputerStatus.js');
let VehicleInfo = require('./VehicleInfo.js');
let BatteryStatus = require('./BatteryStatus.js');
let HilControls = require('./HilControls.js');
let Tunnel = require('./Tunnel.js');
let EstimatorStatus = require('./EstimatorStatus.js');
let GlobalPositionTarget = require('./GlobalPositionTarget.js');
let HomePosition = require('./HomePosition.js');
let DebugValue = require('./DebugValue.js');
let GPSINPUT = require('./GPSINPUT.js');
let HilGPS = require('./HilGPS.js');
let TerrainReport = require('./TerrainReport.js');
let State = require('./State.js');
let StatusText = require('./StatusText.js');
let GPSRTK = require('./GPSRTK.js');
let RCOut = require('./RCOut.js');
let NavControllerOutput = require('./NavControllerOutput.js');
let Param = require('./Param.js');
let CompanionProcessStatus = require('./CompanionProcessStatus.js');
let AttitudeTarget = require('./AttitudeTarget.js');
let Mavlink = require('./Mavlink.js');
let Altitude = require('./Altitude.js');
let ADSBVehicle = require('./ADSBVehicle.js');
let WheelOdomStamped = require('./WheelOdomStamped.js');
let ESCTelemetry = require('./ESCTelemetry.js');
let PlayTuneV2 = require('./PlayTuneV2.js');
let ParamValue = require('./ParamValue.js');
let ManualControl = require('./ManualControl.js');
let LandingTarget = require('./LandingTarget.js');
let CellularStatus = require('./CellularStatus.js');
let CamIMUStamp = require('./CamIMUStamp.js');
let HilSensor = require('./HilSensor.js');
let MagnetometerReporter = require('./MagnetometerReporter.js');
let OverrideRCIn = require('./OverrideRCIn.js');
let Waypoint = require('./Waypoint.js');
let ActuatorControl = require('./ActuatorControl.js');
let MountControl = require('./MountControl.js');
let ExtendedState = require('./ExtendedState.js');
let WaypointList = require('./WaypointList.js');
let Thrust = require('./Thrust.js');
let RTKBaseline = require('./RTKBaseline.js');
let ESCInfoItem = require('./ESCInfoItem.js');
let LogEntry = require('./LogEntry.js');
let CommandCode = require('./CommandCode.js');
let WaypointReached = require('./WaypointReached.js');
let VFR_HUD = require('./VFR_HUD.js');
let Trajectory = require('./Trajectory.js');
let ESCTelemetryItem = require('./ESCTelemetryItem.js');
let FileEntry = require('./FileEntry.js');
let CameraImageCaptured = require('./CameraImageCaptured.js');
let HilActuatorControls = require('./HilActuatorControls.js');
let RTCM = require('./RTCM.js');
let HilStateQuaternion = require('./HilStateQuaternion.js');
let ESCStatusItem = require('./ESCStatusItem.js');
let RadioStatus = require('./RadioStatus.js');
let ESCStatus = require('./ESCStatus.js');
let OpticalFlowRad = require('./OpticalFlowRad.js');
let Vibration = require('./Vibration.js');
let RCIn = require('./RCIn.js');
let LogData = require('./LogData.js');

module.exports = {
  PositionTarget: PositionTarget,
  ESCInfo: ESCInfo,
  GPSRAW: GPSRAW,
  TimesyncStatus: TimesyncStatus,
  OnboardComputerStatus: OnboardComputerStatus,
  VehicleInfo: VehicleInfo,
  BatteryStatus: BatteryStatus,
  HilControls: HilControls,
  Tunnel: Tunnel,
  EstimatorStatus: EstimatorStatus,
  GlobalPositionTarget: GlobalPositionTarget,
  HomePosition: HomePosition,
  DebugValue: DebugValue,
  GPSINPUT: GPSINPUT,
  HilGPS: HilGPS,
  TerrainReport: TerrainReport,
  State: State,
  StatusText: StatusText,
  GPSRTK: GPSRTK,
  RCOut: RCOut,
  NavControllerOutput: NavControllerOutput,
  Param: Param,
  CompanionProcessStatus: CompanionProcessStatus,
  AttitudeTarget: AttitudeTarget,
  Mavlink: Mavlink,
  Altitude: Altitude,
  ADSBVehicle: ADSBVehicle,
  WheelOdomStamped: WheelOdomStamped,
  ESCTelemetry: ESCTelemetry,
  PlayTuneV2: PlayTuneV2,
  ParamValue: ParamValue,
  ManualControl: ManualControl,
  LandingTarget: LandingTarget,
  CellularStatus: CellularStatus,
  CamIMUStamp: CamIMUStamp,
  HilSensor: HilSensor,
  MagnetometerReporter: MagnetometerReporter,
  OverrideRCIn: OverrideRCIn,
  Waypoint: Waypoint,
  ActuatorControl: ActuatorControl,
  MountControl: MountControl,
  ExtendedState: ExtendedState,
  WaypointList: WaypointList,
  Thrust: Thrust,
  RTKBaseline: RTKBaseline,
  ESCInfoItem: ESCInfoItem,
  LogEntry: LogEntry,
  CommandCode: CommandCode,
  WaypointReached: WaypointReached,
  VFR_HUD: VFR_HUD,
  Trajectory: Trajectory,
  ESCTelemetryItem: ESCTelemetryItem,
  FileEntry: FileEntry,
  CameraImageCaptured: CameraImageCaptured,
  HilActuatorControls: HilActuatorControls,
  RTCM: RTCM,
  HilStateQuaternion: HilStateQuaternion,
  ESCStatusItem: ESCStatusItem,
  RadioStatus: RadioStatus,
  ESCStatus: ESCStatus,
  OpticalFlowRad: OpticalFlowRad,
  Vibration: Vibration,
  RCIn: RCIn,
  LogData: LogData,
};
