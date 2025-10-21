// Auto-generated. Do not edit!

// (in-package hardware_node.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class robotHeadMotionData {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.target_position = null;
    }
    else {
      if (initObj.hasOwnProperty('target_position')) {
        this.target_position = initObj.target_position
      }
      else {
        this.target_position = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type robotHeadMotionData
    // Serialize message field [target_position]
    bufferOffset = _arraySerializer.int32(obj.target_position, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type robotHeadMotionData
    let len;
    let data = new robotHeadMotionData(null);
    // Deserialize message field [target_position]
    data.target_position = _arrayDeserializer.int32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.target_position.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'hardware_node/robotHeadMotionData';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '84c8c0833844df50ca703c90b47affbc';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # 机器人头部电机位置 [0, 0]
    int32[] target_position
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new robotHeadMotionData(null);
    if (msg.target_position !== undefined) {
      resolved.target_position = msg.target_position;
    }
    else {
      resolved.target_position = []
    }

    return resolved;
    }
};

module.exports = robotHeadMotionData;
