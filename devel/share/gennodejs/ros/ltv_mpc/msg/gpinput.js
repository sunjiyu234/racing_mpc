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

class gpinput {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.v = null;
      this.r = null;
      this.beta = null;
      this.Tf = null;
      this.Tr = null;
      this.Steer = null;
    }
    else {
      if (initObj.hasOwnProperty('v')) {
        this.v = initObj.v
      }
      else {
        this.v = 0.0;
      }
      if (initObj.hasOwnProperty('r')) {
        this.r = initObj.r
      }
      else {
        this.r = 0.0;
      }
      if (initObj.hasOwnProperty('beta')) {
        this.beta = initObj.beta
      }
      else {
        this.beta = 0.0;
      }
      if (initObj.hasOwnProperty('Tf')) {
        this.Tf = initObj.Tf
      }
      else {
        this.Tf = 0.0;
      }
      if (initObj.hasOwnProperty('Tr')) {
        this.Tr = initObj.Tr
      }
      else {
        this.Tr = 0.0;
      }
      if (initObj.hasOwnProperty('Steer')) {
        this.Steer = initObj.Steer
      }
      else {
        this.Steer = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type gpinput
    // Serialize message field [v]
    bufferOffset = _serializer.float64(obj.v, buffer, bufferOffset);
    // Serialize message field [r]
    bufferOffset = _serializer.float64(obj.r, buffer, bufferOffset);
    // Serialize message field [beta]
    bufferOffset = _serializer.float64(obj.beta, buffer, bufferOffset);
    // Serialize message field [Tf]
    bufferOffset = _serializer.float64(obj.Tf, buffer, bufferOffset);
    // Serialize message field [Tr]
    bufferOffset = _serializer.float64(obj.Tr, buffer, bufferOffset);
    // Serialize message field [Steer]
    bufferOffset = _serializer.float64(obj.Steer, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type gpinput
    let len;
    let data = new gpinput(null);
    // Deserialize message field [v]
    data.v = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [r]
    data.r = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [beta]
    data.beta = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [Tf]
    data.Tf = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [Tr]
    data.Tr = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [Steer]
    data.Steer = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 48;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ltv_mpc/gpinput';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'dfd53f116b4d87a2c94142f47b445347';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 v
    float64 r
    float64 beta
    float64 Tf
    float64 Tr
    float64 Steer
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new gpinput(null);
    if (msg.v !== undefined) {
      resolved.v = msg.v;
    }
    else {
      resolved.v = 0.0
    }

    if (msg.r !== undefined) {
      resolved.r = msg.r;
    }
    else {
      resolved.r = 0.0
    }

    if (msg.beta !== undefined) {
      resolved.beta = msg.beta;
    }
    else {
      resolved.beta = 0.0
    }

    if (msg.Tf !== undefined) {
      resolved.Tf = msg.Tf;
    }
    else {
      resolved.Tf = 0.0
    }

    if (msg.Tr !== undefined) {
      resolved.Tr = msg.Tr;
    }
    else {
      resolved.Tr = 0.0
    }

    if (msg.Steer !== undefined) {
      resolved.Steer = msg.Steer;
    }
    else {
      resolved.Steer = 0.0
    }

    return resolved;
    }
};

module.exports = gpinput;
