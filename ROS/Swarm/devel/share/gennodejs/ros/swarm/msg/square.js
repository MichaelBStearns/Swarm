// Auto-generated. Do not edit!

// (in-package swarm.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class square {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.pheromones = null;
    }
    else {
      if (initObj.hasOwnProperty('pheromones')) {
        this.pheromones = initObj.pheromones
      }
      else {
        this.pheromones = new Array(5).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type square
    // Check that the constant length array field [pheromones] has the right length
    if (obj.pheromones.length !== 5) {
      throw new Error('Unable to serialize array field pheromones - length must be 5')
    }
    // Serialize message field [pheromones]
    bufferOffset = _arraySerializer.char(obj.pheromones, buffer, bufferOffset, 5);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type square
    let len;
    let data = new square(null);
    // Deserialize message field [pheromones]
    data.pheromones = _arrayDeserializer.char(buffer, bufferOffset, 5)
    return data;
  }

  static getMessageSize(object) {
    return 5;
  }

  static datatype() {
    // Returns string type for a message object
    return 'swarm/square';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ad37cb41072571a31d12e95c135b1050';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    char[5] pheromones
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new square(null);
    if (msg.pheromones !== undefined) {
      resolved.pheromones = msg.pheromones;
    }
    else {
      resolved.pheromones = new Array(5).fill(0)
    }

    return resolved;
    }
};

module.exports = square;
