// Auto-generated. Do not edit!

// (in-package swarm.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Square = require('./Square.js');

//-----------------------------------------------------------

class Column {
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
        this.row = new Array(100).fill(new Square());
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Column
    // Check that the constant length array field [row] has the right length
    if (obj.row.length !== 100) {
      throw new Error('Unable to serialize array field row - length must be 100')
    }
    // Serialize message field [row]
    obj.row.forEach((val) => {
      bufferOffset = Square.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Column
    let len;
    let data = new Column(null);
    // Deserialize message field [row]
    len = 100;
    data.row = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.row[i] = Square.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    return 10;
  }

  static datatype() {
    // Returns string type for a message object
    return 'swarm/Column';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ddf6a97d0cfbc524eaea9ad493826b1d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Square[100] row
    ================================================================================
    MSG: swarm/Square
    int16[5] pheromones
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Column(null);
    if (msg.row !== undefined) {
      resolved.row = new Array(100)
      for (let i = 0; i < resolved.row.length; ++i) {
        if (msg.row.length > i) {
          resolved.row[i] = Square.Resolve(msg.row[i]);
        }
        else {
          resolved.row[i] = new Square();
        }
      }
    }
    else {
      resolved.row = new Array(100).fill(new Square())
    }

    return resolved;
    }
};

module.exports = Column;
