// Auto-generated. Do not edit!

// (in-package nav2d_navigator.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class MoveToPosition2DResult {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.final_pose = null;
      this.final_distance = null;
    }
    else {
      if (initObj.hasOwnProperty('final_pose')) {
        this.final_pose = initObj.final_pose
      }
      else {
        this.final_pose = new geometry_msgs.msg.Pose2D();
      }
      if (initObj.hasOwnProperty('final_distance')) {
        this.final_distance = initObj.final_distance
      }
      else {
        this.final_distance = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MoveToPosition2DResult
    // Serialize message field [final_pose]
    bufferOffset = geometry_msgs.msg.Pose2D.serialize(obj.final_pose, buffer, bufferOffset);
    // Serialize message field [final_distance]
    bufferOffset = _serializer.float32(obj.final_distance, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MoveToPosition2DResult
    let len;
    let data = new MoveToPosition2DResult(null);
    // Deserialize message field [final_pose]
    data.final_pose = geometry_msgs.msg.Pose2D.deserialize(buffer, bufferOffset);
    // Deserialize message field [final_distance]
    data.final_distance = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 28;
  }

  static datatype() {
    // Returns string type for a message object
    return 'nav2d_navigator/MoveToPosition2DResult';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1494b1c9041b641e97cee161a63a1b7b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
    geometry_msgs/Pose2D final_pose
    float32 final_distance
    
    ================================================================================
    MSG: geometry_msgs/Pose2D
    # Deprecated
    # Please use the full 3D pose.
    
    # In general our recommendation is to use a full 3D representation of everything and for 2D specific applications make the appropriate projections into the plane for their calculations but optimally will preserve the 3D information during processing.
    
    # If we have parallel copies of 2D datatypes every UI and other pipeline will end up needing to have dual interfaces to plot everything. And you will end up with not being able to use 3D tools for 2D use cases even if they're completely valid, as you'd have to reimplement it with different inputs and outputs. It's not particularly hard to plot the 2D pose or compute the yaw error for the Pose message and there are already tools and libraries that can do this for you.
    
    
    # This expresses a position and orientation on a 2D manifold.
    
    float64 x
    float64 y
    float64 theta
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new MoveToPosition2DResult(null);
    if (msg.final_pose !== undefined) {
      resolved.final_pose = geometry_msgs.msg.Pose2D.Resolve(msg.final_pose)
    }
    else {
      resolved.final_pose = new geometry_msgs.msg.Pose2D()
    }

    if (msg.final_distance !== undefined) {
      resolved.final_distance = msg.final_distance;
    }
    else {
      resolved.final_distance = 0.0
    }

    return resolved;
    }
};

module.exports = MoveToPosition2DResult;
