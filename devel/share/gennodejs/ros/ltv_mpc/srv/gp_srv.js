// Auto-generated. Do not edit!

// (in-package ltv_mpc.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let gpinput = require('../msg/gpinput.js');

//-----------------------------------------------------------

let gpoutput = require('../msg/gpoutput.js');

//-----------------------------------------------------------

class gp_srvRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.gp_input_now = null;
    }
    else {
      if (initObj.hasOwnProperty('gp_input_now')) {
        this.gp_input_now = initObj.gp_input_now
      }
      else {
        this.gp_input_now = new gpinput();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type gp_srvRequest
    // Serialize message field [gp_input_now]
    bufferOffset = gpinput.serialize(obj.gp_input_now, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type gp_srvRequest
    let len;
    let data = new gp_srvRequest(null);
    // Deserialize message field [gp_input_now]
    data.gp_input_now = gpinput.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 48;
  }

  static datatype() {
    // Returns string type for a service object
    return 'ltv_mpc/gp_srvRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6c2b789bc37b75ef90e43a0e84365208';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    gpinput gp_input_now
    
    ================================================================================
    MSG: ltv_mpc/gpinput
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
    const resolved = new gp_srvRequest(null);
    if (msg.gp_input_now !== undefined) {
      resolved.gp_input_now = gpinput.Resolve(msg.gp_input_now)
    }
    else {
      resolved.gp_input_now = new gpinput()
    }

    return resolved;
    }
};

class gp_srvResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.gp_output_now = null;
    }
    else {
      if (initObj.hasOwnProperty('gp_output_now')) {
        this.gp_output_now = initObj.gp_output_now
      }
      else {
        this.gp_output_now = new gpoutput();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type gp_srvResponse
    // Serialize message field [gp_output_now]
    bufferOffset = gpoutput.serialize(obj.gp_output_now, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type gp_srvResponse
    let len;
    let data = new gp_srvResponse(null);
    // Deserialize message field [gp_output_now]
    data.gp_output_now = gpoutput.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 24;
  }

  static datatype() {
    // Returns string type for a service object
    return 'ltv_mpc/gp_srvResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0b3f30157fff2a48622f40da7c543041';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    gpoutput gp_output_now
    
    ================================================================================
    MSG: ltv_mpc/gpoutput
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
    const resolved = new gp_srvResponse(null);
    if (msg.gp_output_now !== undefined) {
      resolved.gp_output_now = gpoutput.Resolve(msg.gp_output_now)
    }
    else {
      resolved.gp_output_now = new gpoutput()
    }

    return resolved;
    }
};

module.exports = {
  Request: gp_srvRequest,
  Response: gp_srvResponse,
  md5sum() { return '44a6e9c0a8d485a5d5e43661bad4e2ba'; },
  datatype() { return 'ltv_mpc/gp_srv'; }
};
