// Auto-generated. Do not edit!

// (in-package msckf.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let FeatureMeasurement = require('./FeatureMeasurement.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class CameraMeasurement {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.features = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('features')) {
        this.features = initObj.features
      }
      else {
        this.features = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CameraMeasurement
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [features]
    // Serialize the length for message field [features]
    bufferOffset = _serializer.uint32(obj.features.length, buffer, bufferOffset);
    obj.features.forEach((val) => {
      bufferOffset = FeatureMeasurement.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CameraMeasurement
    let len;
    let data = new CameraMeasurement(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [features]
    // Deserialize array length for message field [features]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.features = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.features[i] = FeatureMeasurement.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += 48 * object.features.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'msckf/CameraMeasurement';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '03c4d3aa5670b44cd45362d0e5a1400a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    # All features on the current image,
    # including tracked ones and newly detected ones.
    FeatureMeasurement[] features
    
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    ================================================================================
    MSG: msckf/FeatureMeasurement
    uint64 id
    # Normalized feature coordinates (with identity intrinsic matrix)
    uint64 lifetime
    float64 u0 # horizontal coordinate in cam0
    float64 v0 # vertical coordinate in cam0
    float64 u1 # horizontal coordinate in cam0
    float64 v1 # vertical coordinate in cam0
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new CameraMeasurement(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.features !== undefined) {
      resolved.features = new Array(msg.features.length);
      for (let i = 0; i < resolved.features.length; ++i) {
        resolved.features[i] = FeatureMeasurement.Resolve(msg.features[i]);
      }
    }
    else {
      resolved.features = []
    }

    return resolved;
    }
};

module.exports = CameraMeasurement;
