// Auto-generated. Do not edit!

// (in-package pred_msg.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class correct_data {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.index_data = null;
      this.correct_v = null;
      this.correct_r = null;
      this.correct_beta = null;
    }
    else {
      if (initObj.hasOwnProperty('index_data')) {
        this.index_data = initObj.index_data
      }
      else {
        this.index_data = 0;
      }
      if (initObj.hasOwnProperty('correct_v')) {
        this.correct_v = initObj.correct_v
      }
      else {
        this.correct_v = 0.0;
      }
      if (initObj.hasOwnProperty('correct_r')) {
        this.correct_r = initObj.correct_r
      }
      else {
        this.correct_r = 0.0;
      }
      if (initObj.hasOwnProperty('correct_beta')) {
        this.correct_beta = initObj.correct_beta
      }
      else {
        this.correct_beta = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type correct_data
    // Serialize message field [index_data]
    bufferOffset = _serializer.int32(obj.index_data, buffer, bufferOffset);
    // Serialize message field [correct_v]
    bufferOffset = _serializer.float64(obj.correct_v, buffer, bufferOffset);
    // Serialize message field [correct_r]
    bufferOffset = _serializer.float64(obj.correct_r, buffer, bufferOffset);
    // Serialize message field [correct_beta]
    bufferOffset = _serializer.float64(obj.correct_beta, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type correct_data
    let len;
    let data = new correct_data(null);
    // Deserialize message field [index_data]
    data.index_data = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [correct_v]
    data.correct_v = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [correct_r]
    data.correct_r = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [correct_beta]
    data.correct_beta = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 28;
  }

  static datatype() {
    // Returns string type for a message object
    return 'pred_msg/correct_data';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '41f364437c141998133be277ae3d31c7';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 index_data
    float64 correct_v
    float64 correct_r
    float64 correct_beta
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new correct_data(null);
    if (msg.index_data !== undefined) {
      resolved.index_data = msg.index_data;
    }
    else {
      resolved.index_data = 0
    }

    if (msg.correct_v !== undefined) {
      resolved.correct_v = msg.correct_v;
    }
    else {
      resolved.correct_v = 0.0
    }

    if (msg.correct_r !== undefined) {
      resolved.correct_r = msg.correct_r;
    }
    else {
      resolved.correct_r = 0.0
    }

    if (msg.correct_beta !== undefined) {
      resolved.correct_beta = msg.correct_beta;
    }
    else {
      resolved.correct_beta = 0.0
    }

    return resolved;
    }
};

module.exports = correct_data;
