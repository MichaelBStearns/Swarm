// Auto-generated. Do not edit!

// (in-package swarm.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Column = require('./Column.js');

//-----------------------------------------------------------

class Grid {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.column = null;
    }
    else {
      if (initObj.hasOwnProperty('column')) {
        this.column = initObj.column
      }
      else {
        this.column = new Array(100).fill(new Column());
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Grid
    // Check that the constant length array field [column] has the right length
    if (obj.column.length !== 100) {
      throw new Error('Unable to serialize array field column - length must be 100')
    }
    // Serialize message field [column]
    obj.column.forEach((val) => {
      bufferOffset = Column.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Grid
    let len;
    let data = new Grid(null);
    // Deserialize message field [column]
    len = 100;
    data.column = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.column[i] = Column.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    return 10;
  }

  static datatype() {
    // Returns string type for a message object
    return 'swarm/Grid';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '243c44d4957c298207a8acfe78eede41';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Column[100] column
    ================================================================================
    MSG: swarm/Column
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
    const resolved = new Grid(null);
    if (msg.column !== undefined) {
      resolved.column = new Array(100)
      for (let i = 0; i < resolved.column.length; ++i) {
        if (msg.column.length > i) {
          resolved.column[i] = Column.Resolve(msg.column[i]);
        }
        else {
          resolved.column[i] = new Column();
        }
      }
    }
    else {
      resolved.column = new Array(100).fill(new Column())
    }

    return resolved;
    }
};

module.exports = Grid;
