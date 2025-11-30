
"use strict";

let ObjectCount = require('./ObjectCount.js');
let BoundingBoxes = require('./BoundingBoxes.js');
let BoundingBox = require('./BoundingBox.js');
let CheckForObjectsActionGoal = require('./CheckForObjectsActionGoal.js');
let CheckForObjectsResult = require('./CheckForObjectsResult.js');
let CheckForObjectsActionResult = require('./CheckForObjectsActionResult.js');
let CheckForObjectsGoal = require('./CheckForObjectsGoal.js');
let CheckForObjectsFeedback = require('./CheckForObjectsFeedback.js');
let CheckForObjectsAction = require('./CheckForObjectsAction.js');
let CheckForObjectsActionFeedback = require('./CheckForObjectsActionFeedback.js');

module.exports = {
  ObjectCount: ObjectCount,
  BoundingBoxes: BoundingBoxes,
  BoundingBox: BoundingBox,
  CheckForObjectsActionGoal: CheckForObjectsActionGoal,
  CheckForObjectsResult: CheckForObjectsResult,
  CheckForObjectsActionResult: CheckForObjectsActionResult,
  CheckForObjectsGoal: CheckForObjectsGoal,
  CheckForObjectsFeedback: CheckForObjectsFeedback,
  CheckForObjectsAction: CheckForObjectsAction,
  CheckForObjectsActionFeedback: CheckForObjectsActionFeedback,
};
