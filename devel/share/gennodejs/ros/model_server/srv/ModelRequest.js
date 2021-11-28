// Auto-generated. Do not edit!

// (in-package model_server.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

let InferenceResults = require('../msg/InferenceResults.js');

//-----------------------------------------------------------

class ModelRequestRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.model = null;
    }
    else {
      if (initObj.hasOwnProperty('model')) {
        this.model = initObj.model
      }
      else {
        this.model = new std_msgs.msg.String();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ModelRequestRequest
    // Serialize message field [model]
    bufferOffset = std_msgs.msg.String.serialize(obj.model, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ModelRequestRequest
    let len;
    let data = new ModelRequestRequest(null);
    // Deserialize message field [model]
    data.model = std_msgs.msg.String.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.String.getMessageSize(object.model);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'model_server/ModelRequestRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '7f3dcb935694565eda084843802a1ee4';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/String model
    
    ================================================================================
    MSG: std_msgs/String
    string data
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ModelRequestRequest(null);
    if (msg.model !== undefined) {
      resolved.model = std_msgs.msg.String.Resolve(msg.model)
    }
    else {
      resolved.model = new std_msgs.msg.String()
    }

    return resolved;
    }
};

class ModelRequestResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.results = null;
    }
    else {
      if (initObj.hasOwnProperty('results')) {
        this.results = initObj.results
      }
      else {
        this.results = new InferenceResults();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ModelRequestResponse
    // Serialize message field [results]
    bufferOffset = InferenceResults.serialize(obj.results, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ModelRequestResponse
    let len;
    let data = new ModelRequestResponse(null);
    // Deserialize message field [results]
    data.results = InferenceResults.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += InferenceResults.getMessageSize(object.results);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'model_server/ModelRequestResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6af076944b82152d7ca8f015f76ac624';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    InferenceResults    results
    
    
    ================================================================================
    MSG: model_server/InferenceResults
    int32[] structure
    float32[]   inferences
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ModelRequestResponse(null);
    if (msg.results !== undefined) {
      resolved.results = InferenceResults.Resolve(msg.results)
    }
    else {
      resolved.results = new InferenceResults()
    }

    return resolved;
    }
};

module.exports = {
  Request: ModelRequestRequest,
  Response: ModelRequestResponse,
  md5sum() { return '84e67d26165d3d65758a56814ed143ba'; },
  datatype() { return 'model_server/ModelRequest'; }
};
