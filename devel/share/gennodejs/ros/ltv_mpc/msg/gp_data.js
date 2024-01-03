// Auto-generated. Do not edit!

// (in-package ltv_mpc.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let gpinput = require('./gpinput.js');
let gpoutput = require('./gpoutput.js');

//-----------------------------------------------------------

class gp_data {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.gp_input = null;
      this.gp_output = null;
    }
    else {
      if (initObj.hasOwnProperty('gp_input')) {
        this.gp_input = initObj.gp_input
      }
      else {
        this.gp_input = new gpinput();
      }
      if (initObj.hasOwnProperty('gp_output')) {
        this.gp_output = initObj.gp_output
      }
      else {
        this.gp_output = new gpoutput();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type gp_data
    // Serialize message field [gp_input]
    bufferOffset = gpinput.serialize(obj.gp_input, buffer, bufferOffset);
    // Serialize message field [gp_output]
    bufferOffset = gpoutput.serialize(obj.gp_output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type gp_data
    let len;
    let data = new gp_data(null);
    // Deserialize message field [gp_input]
    data.gp_input = gpinput.deserialize(buffer, bufferOffset);
    // Deserialize message field [gp_output]
    data.gp_output = gpoutput.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 72;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ltv_mpc/gp_data';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2757ea2368b9f205bb07f6799989923c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    gpinput gp_input
    gpoutput gp_output
    ================================================================================
    MSG: ltv_mpc/gpinput
    float64 v
    float64 r
    float64 beta
    float64 Tf
    float64 Tr
    float64 Steer
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
    const resolved = new gp_data(null);
    if (msg.gp_input !== undefined) {
      resolved.gp_input = gpinput.Resolve(msg.gp_input)
    }
    else {
      resolved.gp_input = new gpinput()
    }

    if (msg.gp_output !== undefined) {
      resolved.gp_output = gpoutput.Resolve(msg.gp_output)
    }
    else {
      resolved.gp_output = new gpoutput()
    }

    return resolved;
    }
};

module.exports = gp_data;
