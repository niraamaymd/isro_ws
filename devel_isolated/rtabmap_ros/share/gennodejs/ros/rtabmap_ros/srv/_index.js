
"use strict";

let GetNodeData = require('./GetNodeData.js')
let GetNodesInRadius = require('./GetNodesInRadius.js')
let GetPlan = require('./GetPlan.js')
let AddLink = require('./AddLink.js')
let GetMap2 = require('./GetMap2.js')
let GetMap = require('./GetMap.js')
let PublishMap = require('./PublishMap.js')
let SetGoal = require('./SetGoal.js')
let LoadDatabase = require('./LoadDatabase.js')
let SetLabel = require('./SetLabel.js')
let ResetPose = require('./ResetPose.js')
let ListLabels = require('./ListLabels.js')

module.exports = {
  GetNodeData: GetNodeData,
  GetNodesInRadius: GetNodesInRadius,
  GetPlan: GetPlan,
  AddLink: AddLink,
  GetMap2: GetMap2,
  GetMap: GetMap,
  PublishMap: PublishMap,
  SetGoal: SetGoal,
  LoadDatabase: LoadDatabase,
  SetLabel: SetLabel,
  ResetPose: ResetPose,
  ListLabels: ListLabels,
};
