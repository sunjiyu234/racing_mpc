// Auto-generated. Do not edit!

// (in-package ltv_mpc.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class gpoutput {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.v_error = null;
      this.r_error = null;
      this.beta_error = null;
    }
    else {
      if (initObj.hasOwnProperty('v_error')) {
        this.v_error = initObj.v_error
      }
      else {
        this.v_error = 0.0;
      }
      if (initObj.hasOwnProperty('r_error')) {
        this.r_error = initObj.r_error
      }
      else {
        this.r_error = 0.0;
      }
      if (initObj.hasOwnProperty('beta_error')) {
        this.beta_error = initObj.beta_error
      }
      else {
        this.beta_error = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type gpoutput
    // Serialize message field [v_error]
    bufferOffset = _serializer.float64(obj.v_error, buffer, bufferOffset);
    // Serialize message field [r_error]
    bufferOffset = _serializer.float64(obj.r_error, buffer, bufferOffset);
    // Serialize message field [beta_error]
    bufferOffset = _serializer.float64(obj.beta_error, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type gpoutput
    let len;
    let data = new gpoutput(null);
    // Deserialize message field [v_error]
    data.v_error = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [r_error]
    data.r_error = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [beta_error]
    data.beta_error = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 24;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ltv_mpc/gpoutput';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '529d1c55d988fb7bb8a7cea9ad6453f5';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 v_error
    float64 r_error
    float64 beta_error
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new gpoutput(null);
    if (msg.v_error !== undefined) {
      resolved.v_error = msg.v_error;
    }
    else {
      resolved.v_error = 0.0
    }

    if (msg.r_error !== undefined) {
      resolved.r_error = msg.r_error;
    }
    else {
      resolved.r_error = 0.0
    }

    if (msg.beta_error !== undefined) {
      resolved.beta_error = msg.beta_error;
    }
    else {
      resolved.beta_error = 0.0
    }

    return resolved;
    }
};

module.exports = gpoutput;
