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

class column {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.row = null;
    }
    else {
      if (initObj.hasOwnProperty('row')) {
        this.row = initObj.row
      }
      else {
        this.row = new Array(100).fill(new square());
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type column
    // Check that the constant length array field [row] has the right length
    if (obj.row.length !== 100) {
      throw new Error('Unable to serialize array field row - length must be 100')
    }
    // Serialize message field [row]
    obj.row.forEach((val) => {
      bufferOffset = square.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type column
    let len;
    let data = new column(null);
    // Deserialize message field [row]
    len = 100;
    data.row = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.row[i] = square.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    return 5;
  }

  static datatype() {
    // Returns string type for a message object
    return 'swarm/column';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3606c5dc7d55d80c61e5a77e52f55b69';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    square[100] row
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
    const resolved = new column(null);
    if (msg.row !== undefined) {
      resolved.row = new Array(100)
      for (let i = 0; i < resolved.row.length; ++i) {
        if (msg.row.length > i) {
          resolved.row[i] = square.Resolve(msg.row[i]);
        }
        else {
          resolved.row[i] = new square();
        }
      }
    }
    else {
      resolved.row = new Array(100).fill(new square())
    }

    return resolved;
    }
};

module.exports = column;
