
"use strict";

let GetRobotMode = require('./GetRobotMode.js')
let Popup = require('./Popup.js')
let Load = require('./Load.js')
let IsProgramRunning = require('./IsProgramRunning.js')
let IsProgramSaved = require('./IsProgramSaved.js')
let GetSafetyMode = require('./GetSafetyMode.js')
let RawRequest = require('./RawRequest.js')
let IsInRemoteControl = require('./IsInRemoteControl.js')
let AddToLog = require('./AddToLog.js')
let GetLoadedProgram = require('./GetLoadedProgram.js')
let GetProgramState = require('./GetProgramState.js')

module.exports = {
  GetRobotMode: GetRobotMode,
  Popup: Popup,
  Load: Load,
  IsProgramRunning: IsProgramRunning,
  IsProgramSaved: IsProgramSaved,
  GetSafetyMode: GetSafetyMode,
  RawRequest: RawRequest,
  IsInRemoteControl: IsInRemoteControl,
  AddToLog: AddToLog,
  GetLoadedProgram: GetLoadedProgram,
  GetProgramState: GetProgramState,
};
