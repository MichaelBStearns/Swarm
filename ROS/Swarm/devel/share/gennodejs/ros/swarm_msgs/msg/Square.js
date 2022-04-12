// Auto-generated. Do not edit!

// (in-package swarm_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Square {
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
    // Serializes a message object of type Square
    // Check that the constant length array field [pheromones] has the right length
    if (obj.pheromones.length !== 5) {
      throw new Error('Unable to serialize array field pheromones - length must be 5')
    }
    // Serialize message field [pheromones]
    bufferOffset = _arraySerializer.int8(obj.pheromones, buffer, bufferOffset, 5);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Square
    let len;
    let data = new Square(null);
    // Deserialize message field [pheromones]
    data.pheromones = _arrayDeserializer.int8(buffer, bufferOffset, 5)
    return data;
  }

  static getMessageSize(object) {
    return 5;
  }

  static datatype() {
    // Returns string type for a message object
    return 'swarm_msgs/Square';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '98b485e50aa2d6a27ada82a8008715c5';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int8[5] pheromones
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Square(null);
    if (msg.pheromones !== undefined) {
      resolved.pheromones = msg.pheromones;
    }
    else {
      resolved.pheromones = new Array(5).fill(0)
    }

    return resolved;
    }
};

module.exports = Square;
