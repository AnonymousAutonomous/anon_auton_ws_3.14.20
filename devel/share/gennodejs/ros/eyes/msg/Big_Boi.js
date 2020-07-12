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

class Big_Boi {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.timmy = null;
      this.tommy = null;
      this.tammy = null;
    }
    else {
      if (initObj.hasOwnProperty('timmy')) {
        this.timmy = initObj.timmy
      }
      else {
        this.timmy = false;
      }
      if (initObj.hasOwnProperty('tommy')) {
        this.tommy = initObj.tommy
      }
      else {
        this.tommy = false;
      }
      if (initObj.hasOwnProperty('tammy')) {
        this.tammy = initObj.tammy
      }
      else {
        this.tammy = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Big_Boi
    // Serialize message field [timmy]
    bufferOffset = _serializer.bool(obj.timmy, buffer, bufferOffset);
    // Serialize message field [tommy]
    bufferOffset = _serializer.bool(obj.tommy, buffer, bufferOffset);
    // Serialize message field [tammy]
    bufferOffset = _serializer.bool(obj.tammy, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Big_Boi
    let len;
    let data = new Big_Boi(null);
    // Deserialize message field [timmy]
    data.timmy = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [tommy]
    data.tommy = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [tammy]
    data.tammy = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 3;
  }

  static datatype() {
    // Returns string type for a message object
    return 'eyes/Big_Boi';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '14735f0ab9bd89e9e2ae5c4a9db3ab4c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool timmy
    bool tommy
    bool tammy
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Big_Boi(null);
    if (msg.timmy !== undefined) {
      resolved.timmy = msg.timmy;
    }
    else {
      resolved.timmy = false
    }

    if (msg.tommy !== undefined) {
      resolved.tommy = msg.tommy;
    }
    else {
      resolved.tommy = false
    }

    if (msg.tammy !== undefined) {
      resolved.tammy = msg.tammy;
    }
    else {
      resolved.tammy = false
    }

    return resolved;
    }
};

module.exports = Big_Boi;
