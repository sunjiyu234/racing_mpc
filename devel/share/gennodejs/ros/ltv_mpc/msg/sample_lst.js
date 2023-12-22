// Auto-generated. Do not edit!

// (in-package ltv_mpc.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let sample = require('./sample.js');

//-----------------------------------------------------------

class sample_lst {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.sample_list = null;
    }
    else {
      if (initObj.hasOwnProperty('sample_list')) {
        this.sample_list = initObj.sample_list
      }
      else {
        this.sample_list = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type sample_lst
    // Serialize message field [sample_list]
    // Serialize the length for message field [sample_list]
    bufferOffset = _serializer.uint32(obj.sample_list.length, buffer, bufferOffset);
    obj.sample_list.forEach((val) => {
      bufferOffset = sample.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type sample_lst
    let len;
    let data = new sample_lst(null);
    // Deserialize message field [sample_list]
    // Deserialize array length for message field [sample_list]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.sample_list = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.sample_list[i] = sample.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 104 * object.sample_list.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ltv_mpc/sample_lst';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4b7f364dc273676e8884e28e28f4d153';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    sample[] sample_list
    ================================================================================
    MSG: ltv_mpc/sample
    float64 x
    float64 y
    float64 yaw
    float64 v
    float64 r
    float64 beta
    float64 tf
    float64 tr
    float64 steer
    float64 s
    float64 time
    float64 iter
    float64 cost
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new sample_lst(null);
    if (msg.sample_list !== undefined) {
      resolved.sample_list = new Array(msg.sample_list.length);
      for (let i = 0; i < resolved.sample_list.length; ++i) {
        resolved.sample_list[i] = sample.Resolve(msg.sample_list[i]);
      }
    }
    else {
      resolved.sample_list = []
    }

    return resolved;
    }
};

module.exports = sample_lst;
