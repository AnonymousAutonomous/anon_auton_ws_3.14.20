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

class Choreo {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.time = null;
      this.duration = null;
      this.left_forward = null;
      this.right_forward = null;
      this.left_speed = null;
      this.right_speed = null;
    }
    else {
      if (initObj.hasOwnProperty('time')) {
        this.time = initObj.time
      }
      else {
        this.time = false;
      }
      if (initObj.hasOwnProperty('duration')) {
        this.duration = initObj.duration
      }
      else {
        this.duration = 0;
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
    // Serializes a message object of type Choreo
    // Serialize message field [time]
    bufferOffset = _serializer.bool(obj.time, buffer, bufferOffset);
    // Serialize message field [duration]
    bufferOffset = _serializer.int32(obj.duration, buffer, bufferOffset);
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
    //deserializes a message object of type Choreo
    let len;
    let data = new Choreo(null);
    // Deserialize message field [time]
    data.time = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [duration]
    data.duration = _deserializer.int32(buffer, bufferOffset);
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
    return 11;
  }

  static datatype() {
    // Returns string type for a message object
    return 'eyes/Choreo';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1c9b477594bac2435715c7e4261d26c8';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool time
    int32 duration
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
    const resolved = new Choreo(null);
    if (msg.time !== undefined) {
      resolved.time = msg.time;
    }
    else {
      resolved.time = false
    }

    if (msg.duration !== undefined) {
      resolved.duration = msg.duration;
    }
    else {
      resolved.duration = 0
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

module.exports = Choreo;
