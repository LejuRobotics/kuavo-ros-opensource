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

class horizontalMotionByStep {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.step = null;
    }
    else {
      if (initObj.hasOwnProperty('step')) {
        this.step = initObj.step
      }
      else {
        this.step = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type horizontalMotionByStep
    // Serialize message field [step]
    bufferOffset = _serializer.int32(obj.step, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type horizontalMotionByStep
    let len;
    let data = new horizontalMotionByStep(null);
    // Deserialize message field [step]
    data.step = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'hardware_node/horizontalMotionByStep';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '99174260c0c07917ce2b7a46302ab7a8';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # 单步左右转动，传入值为转动的步长
    int32 step
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new horizontalMotionByStep(null);
    if (msg.step !== undefined) {
      resolved.step = msg.step;
    }
    else {
      resolved.step = 0
    }

    return resolved;
    }
};

module.exports = horizontalMotionByStep;
