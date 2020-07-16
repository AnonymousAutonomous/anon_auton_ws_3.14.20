// Auto-generated. Do not edit!

// (in-package eyes.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Generic {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.identifier = null;
      this.left_forward = null;
      this.right_forward = null;
      this.left_speed = null;
      this.right_speed = null;
      this.timed = null;
      this.duration = null;
    }
    else {
      if (initObj.hasOwnProperty('identifier')) {
        this.identifier = initObj.identifier
      }
      else {
        this.identifier = 0;
      }
      if (initObj.hasOwnProperty('left_forward')) {
        this.left_forward = initObj.left_forward
      }
      else {
        this.left_forward = false;
      }
      if (initObj.hasOwnProperty('right_forward')) {
        this.right_forward = initObj.right_forward
      }
      else {
        this.right_forward = false;
      }
      if (initObj.hasOwnProperty('left_speed')) {
        this.left_speed = initObj.left_speed
      }
      else {
        this.left_speed = 0;
      }
      if (initObj.hasOwnProperty('right_speed')) {
        this.right_speed = initObj.right_speed
      }
      else {
        this.right_speed = 0;
      }
      if (initObj.hasOwnProperty('timed')) {
        this.timed = initObj.timed
      }
      else {
        this.timed = false;
      }
      if (initObj.hasOwnProperty('duration')) {
        this.duration = initObj.duration
      }
      else {
        this.duration = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Generic
    // Serialize message field [identifier]
    bufferOffset = _serializer.char(obj.identifier, buffer, bufferOffset);
    // Serialize message field [left_forward]
    bufferOffset = _serializer.bool(obj.left_forward, buffer, bufferOffset);
    // Serialize message field [right_forward]
    bufferOffset = _serializer.bool(obj.right_forward, buffer, bufferOffset);
    // Serialize message field [left_speed]
    bufferOffset = _serializer.uint8(obj.left_speed, buffer, bufferOffset);
    // Serialize message field [right_speed]
    bufferOffset = _serializer.uint8(obj.right_speed, buffer, bufferOffset);
    // Serialize message field [timed]
    bufferOffset = _serializer.bool(obj.timed, buffer, bufferOffset);
    // Serialize message field [duration]
    bufferOffset = _serializer.uint32(obj.duration, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Generic
    let len;
    let data = new Generic(null);
    // Deserialize message field [identifier]
    data.identifier = _deserializer.char(buffer, bufferOffset);
    // Deserialize message field [left_forward]
    data.left_forward = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [right_forward]
    data.right_forward = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [left_speed]
    data.left_speed = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [right_speed]
    data.right_speed = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [timed]
    data.timed = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [duration]
    data.duration = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 10;
  }

  static datatype() {
    // Returns string type for a message object
    return 'eyes/Generic';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '7a33c669a022f7fea29ccba33d517b1f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    char identifier
    bool left_forward
    bool right_forward
    uint8 left_speed
    uint8 right_speed
    bool timed
    uint32 duration
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Generic(null);
    if (msg.identifier !== undefined) {
      resolved.identifier = msg.identifier;
    }
    else {
      resolved.identifier = 0
    }

    if (msg.left_forward !== undefined) {
      resolved.left_forward = msg.left_forward;
    }
    else {
      resolved.left_forward = false
    }

    if (msg.right_forward !== undefined) {
      resolved.right_forward = msg.right_forward;
    }
    else {
      resolved.right_forward = false
    }

    if (msg.left_speed !== undefined) {
      resolved.left_speed = msg.left_speed;
    }
    else {
      resolved.left_speed = 0
    }

    if (msg.right_speed !== undefined) {
      resolved.right_speed = msg.right_speed;
    }
    else {
      resolved.right_speed = 0
    }

    if (msg.timed !== undefined) {
      resolved.timed = msg.timed;
    }
    else {
      resolved.timed = false
    }

    if (msg.duration !== undefined) {
      resolved.duration = msg.duration;
    }
    else {
      resolved.duration = 0
    }

    return resolved;
    }
};

module.exports = Generic;
