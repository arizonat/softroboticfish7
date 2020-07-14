// Auto-generated. Do not edit!

// (in-package fishstatecontroller.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Position {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.distance = null;
      this.x_offset = null;
      this.y_offset = null;
    }
    else {
      if (initObj.hasOwnProperty('distance')) {
        this.distance = initObj.distance
      }
      else {
        this.distance = '';
      }
      if (initObj.hasOwnProperty('x_offset')) {
        this.x_offset = initObj.x_offset
      }
      else {
        this.x_offset = '';
      }
      if (initObj.hasOwnProperty('y_offset')) {
        this.y_offset = initObj.y_offset
      }
      else {
        this.y_offset = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Position
    // Serialize message field [distance]
    bufferOffset = _serializer.string(obj.distance, buffer, bufferOffset);
    // Serialize message field [x_offset]
    bufferOffset = _serializer.string(obj.x_offset, buffer, bufferOffset);
    // Serialize message field [y_offset]
    bufferOffset = _serializer.string(obj.y_offset, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Position
    let len;
    let data = new Position(null);
    // Deserialize message field [distance]
    data.distance = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [x_offset]
    data.x_offset = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [y_offset]
    data.y_offset = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.distance.length;
    length += object.x_offset.length;
    length += object.y_offset.length;
    return length + 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'fishstatecontroller/Position';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '7269bbef7b024d4726181ff666ab9ee9';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string distance
    string x_offset
    string y_offset
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Position(null);
    if (msg.distance !== undefined) {
      resolved.distance = msg.distance;
    }
    else {
      resolved.distance = ''
    }

    if (msg.x_offset !== undefined) {
      resolved.x_offset = msg.x_offset;
    }
    else {
      resolved.x_offset = ''
    }

    if (msg.y_offset !== undefined) {
      resolved.y_offset = msg.y_offset;
    }
    else {
      resolved.y_offset = ''
    }

    return resolved;
    }
};

module.exports = Position;
