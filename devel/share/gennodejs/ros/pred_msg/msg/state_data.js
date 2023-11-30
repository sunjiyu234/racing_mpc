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

class state_data {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.index_data = null;
      this.input_v = null;
      this.input_r = null;
      this.input_beta = null;
      this.input_Tf = null;
      this.input_Tr = null;
      this.input_delta = null;
      this.input_error_v = null;
      this.input_error_r = null;
      this.input_error_beta = null;
    }
    else {
      if (initObj.hasOwnProperty('index_data')) {
        this.index_data = initObj.index_data
      }
      else {
        this.index_data = 0;
      }
      if (initObj.hasOwnProperty('input_v')) {
        this.input_v = initObj.input_v
      }
      else {
        this.input_v = 0.0;
      }
      if (initObj.hasOwnProperty('input_r')) {
        this.input_r = initObj.input_r
      }
      else {
        this.input_r = 0.0;
      }
      if (initObj.hasOwnProperty('input_beta')) {
        this.input_beta = initObj.input_beta
      }
      else {
        this.input_beta = 0.0;
      }
      if (initObj.hasOwnProperty('input_Tf')) {
        this.input_Tf = initObj.input_Tf
      }
      else {
        this.input_Tf = 0.0;
      }
      if (initObj.hasOwnProperty('input_Tr')) {
        this.input_Tr = initObj.input_Tr
      }
      else {
        this.input_Tr = 0.0;
      }
      if (initObj.hasOwnProperty('input_delta')) {
        this.input_delta = initObj.input_delta
      }
      else {
        this.input_delta = 0.0;
      }
      if (initObj.hasOwnProperty('input_error_v')) {
        this.input_error_v = initObj.input_error_v
      }
      else {
        this.input_error_v = 0.0;
      }
      if (initObj.hasOwnProperty('input_error_r')) {
        this.input_error_r = initObj.input_error_r
      }
      else {
        this.input_error_r = 0.0;
      }
      if (initObj.hasOwnProperty('input_error_beta')) {
        this.input_error_beta = initObj.input_error_beta
      }
      else {
        this.input_error_beta = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type state_data
    // Serialize message field [index_data]
    bufferOffset = _serializer.int32(obj.index_data, buffer, bufferOffset);
    // Serialize message field [input_v]
    bufferOffset = _serializer.float64(obj.input_v, buffer, bufferOffset);
    // Serialize message field [input_r]
    bufferOffset = _serializer.float64(obj.input_r, buffer, bufferOffset);
    // Serialize message field [input_beta]
    bufferOffset = _serializer.float64(obj.input_beta, buffer, bufferOffset);
    // Serialize message field [input_Tf]
    bufferOffset = _serializer.float64(obj.input_Tf, buffer, bufferOffset);
    // Serialize message field [input_Tr]
    bufferOffset = _serializer.float64(obj.input_Tr, buffer, bufferOffset);
    // Serialize message field [input_delta]
    bufferOffset = _serializer.float64(obj.input_delta, buffer, bufferOffset);
    // Serialize message field [input_error_v]
    bufferOffset = _serializer.float64(obj.input_error_v, buffer, bufferOffset);
    // Serialize message field [input_error_r]
    bufferOffset = _serializer.float64(obj.input_error_r, buffer, bufferOffset);
    // Serialize message field [input_error_beta]
    bufferOffset = _serializer.float64(obj.input_error_beta, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type state_data
    let len;
    let data = new state_data(null);
    // Deserialize message field [index_data]
    data.index_data = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [input_v]
    data.input_v = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [input_r]
    data.input_r = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [input_beta]
    data.input_beta = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [input_Tf]
    data.input_Tf = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [input_Tr]
    data.input_Tr = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [input_delta]
    data.input_delta = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [input_error_v]
    data.input_error_v = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [input_error_r]
    data.input_error_r = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [input_error_beta]
    data.input_error_beta = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 76;
  }

  static datatype() {
    // Returns string type for a message object
    return 'pred_msg/state_data';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'daae5795cc22390ee753757c10fc1367';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 index_data
    float64 input_v
    float64 input_r
    float64 input_beta
    float64 input_Tf
    float64 input_Tr
    float64 input_delta
    float64 input_error_v
    float64 input_error_r
    float64 input_error_beta
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new state_data(null);
    if (msg.index_data !== undefined) {
      resolved.index_data = msg.index_data;
    }
    else {
      resolved.index_data = 0
    }

    if (msg.input_v !== undefined) {
      resolved.input_v = msg.input_v;
    }
    else {
      resolved.input_v = 0.0
    }

    if (msg.input_r !== undefined) {
      resolved.input_r = msg.input_r;
    }
    else {
      resolved.input_r = 0.0
    }

    if (msg.input_beta !== undefined) {
      resolved.input_beta = msg.input_beta;
    }
    else {
      resolved.input_beta = 0.0
    }

    if (msg.input_Tf !== undefined) {
      resolved.input_Tf = msg.input_Tf;
    }
    else {
      resolved.input_Tf = 0.0
    }

    if (msg.input_Tr !== undefined) {
      resolved.input_Tr = msg.input_Tr;
    }
    else {
      resolved.input_Tr = 0.0
    }

    if (msg.input_delta !== undefined) {
      resolved.input_delta = msg.input_delta;
    }
    else {
      resolved.input_delta = 0.0
    }

    if (msg.input_error_v !== undefined) {
      resolved.input_error_v = msg.input_error_v;
    }
    else {
      resolved.input_error_v = 0.0
    }

    if (msg.input_error_r !== undefined) {
      resolved.input_error_r = msg.input_error_r;
    }
    else {
      resolved.input_error_r = 0.0
    }

    if (msg.input_error_beta !== undefined) {
      resolved.input_error_beta = msg.input_error_beta;
    }
    else {
      resolved.input_error_beta = 0.0
    }

    return resolved;
    }
};

module.exports = state_data;
