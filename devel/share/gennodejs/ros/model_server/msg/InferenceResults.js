// Auto-generated. Do not edit!

// (in-package model_server.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class InferenceResults {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.structure = null;
      this.inferences = null;
    }
    else {
      if (initObj.hasOwnProperty('structure')) {
        this.structure = initObj.structure
      }
      else {
        this.structure = [];
      }
      if (initObj.hasOwnProperty('inferences')) {
        this.inferences = initObj.inferences
      }
      else {
        this.inferences = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type InferenceResults
    // Serialize message field [structure]
    bufferOffset = _arraySerializer.int32(obj.structure, buffer, bufferOffset, null);
    // Serialize message field [inferences]
    bufferOffset = _arraySerializer.float32(obj.inferences, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type InferenceResults
    let len;
    let data = new InferenceResults(null);
    // Deserialize message field [structure]
    data.structure = _arrayDeserializer.int32(buffer, bufferOffset, null)
    // Deserialize message field [inferences]
    data.inferences = _arrayDeserializer.float32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.structure.length;
    length += 4 * object.inferences.length;
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'model_server/InferenceResults';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '794290d6a80514eaeded7ed9f07c9d16';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32[] structure
    float32[]   inferences
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new InferenceResults(null);
    if (msg.structure !== undefined) {
      resolved.structure = msg.structure;
    }
    else {
      resolved.structure = []
    }

    if (msg.inferences !== undefined) {
      resolved.inferences = msg.inferences;
    }
    else {
      resolved.inferences = []
    }

    return resolved;
    }
};

module.exports = InferenceResults;
