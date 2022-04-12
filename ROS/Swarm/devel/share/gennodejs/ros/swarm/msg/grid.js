// Auto-generated. Do not edit!

// (in-package swarm.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let square = require('./square.js');

//-----------------------------------------------------------

class grid {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.box = null;
    }
    else {
      if (initObj.hasOwnProperty('box')) {
        this.box = initObj.box
      }
      else {
        this.box = new square();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type grid
    // Serialize message field [box]
    bufferOffset = square.serialize(obj.box, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type grid
    let len;
    let data = new grid(null);
    // Deserialize message field [box]
    data.box = square.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 5;
  }

  static datatype() {
    // Returns string type for a message object
    return 'swarm/grid';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '32a280d72cb6e8c663e8b4d302b71970';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    square box
    ================================================================================
    MSG: swarm/square
    char[5] pheromones
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new grid(null);
    if (msg.box !== undefined) {
      resolved.box = square.Resolve(msg.box)
    }
    else {
      resolved.box = new square()
    }

    return resolved;
    }
};

module.exports = grid;
