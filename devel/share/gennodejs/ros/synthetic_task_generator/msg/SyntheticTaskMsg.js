// Auto-generated. Do not edit!

// (in-package synthetic_task_generator.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class SyntheticTaskMsg {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.instance = null;
      this.value = null;
    }
    else {
      if (initObj.hasOwnProperty('instance')) {
        this.instance = initObj.instance
      }
      else {
        this.instance = 0;
      }
      if (initObj.hasOwnProperty('value')) {
        this.value = initObj.value
      }
      else {
        this.value = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SyntheticTaskMsg
    // Serialize message field [instance]
    bufferOffset = _serializer.int64(obj.instance, buffer, bufferOffset);
    // Serialize message field [value]
    bufferOffset = _serializer.float64(obj.value, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SyntheticTaskMsg
    let len;
    let data = new SyntheticTaskMsg(null);
    // Deserialize message field [instance]
    data.instance = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [value]
    data.value = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'synthetic_task_generator/SyntheticTaskMsg';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'cbfe4c158f332960f1182cd0ed832cbc';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int64 instance
    float64 value
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SyntheticTaskMsg(null);
    if (msg.instance !== undefined) {
      resolved.instance = msg.instance;
    }
    else {
      resolved.instance = 0
    }

    if (msg.value !== undefined) {
      resolved.value = msg.value;
    }
    else {
      resolved.value = 0.0
    }

    return resolved;
    }
};

module.exports = SyntheticTaskMsg;
