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

class Autonomous {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.safety = null;
      this.left_forward = null;
      this.right_forward = null;
      this.left_speed = null;
      this.right_speed = null;
    }
    else {
      if (initObj.hasOwnProperty('safety')) {
        this.safety = initObj.safety
      }
      else {
        this.safety = false;
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
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Autonomous
    // Serialize message field [safety]
    bufferOffset = _serializer.bool(obj.safety, buffer, bufferOffset);
    // Serialize message field [left_forward]
    bufferOffset = _serializer.bool(obj.left_forward, buffer, bufferOffset);
    // Serialize message field [right_forward]
    bufferOffset = _serializer.bool(obj.right_forward, buffer, bufferOffset);
    // Serialize message field [left_speed]
    bufferOffset = _serializer.int16(obj.left_speed, buffer, bufferOffset);
    // Serialize message field [right_speed]
    bufferOffset = _serializer.int16(obj.right_speed, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Autonomous
    let len;
    let data = new Autonomous(null);
    // Deserialize message field [safety]
    data.safety = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [left_forward]
    data.left_forward = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [right_forward]
    data.right_forward = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [left_speed]
    data.left_speed = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [right_speed]
    data.right_speed = _deserializer.int16(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 7;
  }

  static datatype() {
    // Returns string type for a message object
    return 'eyes/Autonomous';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4cbf5bb4d4e0610600c1d61a65d9f85d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool safety
    bool left_forward
    bool right_forward
    int16 left_speed
    int16 right_speed
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Autonomous(null);
    if (msg.safety !== undefined) {
      resolved.safety = msg.safety;
    }
    else {
      resolved.safety = false
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

    return resolved;
    }
};

module.exports = Autonomous;
