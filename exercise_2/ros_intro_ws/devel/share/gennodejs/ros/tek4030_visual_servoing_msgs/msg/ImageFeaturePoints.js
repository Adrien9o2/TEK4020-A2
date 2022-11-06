// Auto-generated. Do not edit!

// (in-package tek4030_visual_servoing_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class ImageFeaturePoints {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.p = null;
    }
    else {
      if (initObj.hasOwnProperty('p')) {
        this.p = initObj.p
      }
      else {
        this.p = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ImageFeaturePoints
    // Serialize message field [p]
    // Serialize the length for message field [p]
    bufferOffset = _serializer.uint32(obj.p.length, buffer, bufferOffset);
    obj.p.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Point.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ImageFeaturePoints
    let len;
    let data = new ImageFeaturePoints(null);
    // Deserialize message field [p]
    // Deserialize array length for message field [p]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.p = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.p[i] = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 24 * object.p.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'tek4030_visual_servoing_msgs/ImageFeaturePoints';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '74ca570665a40606dd930ab3deca3e02';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    geometry_msgs/Point[] p
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ImageFeaturePoints(null);
    if (msg.p !== undefined) {
      resolved.p = new Array(msg.p.length);
      for (let i = 0; i < resolved.p.length; ++i) {
        resolved.p[i] = geometry_msgs.msg.Point.Resolve(msg.p[i]);
      }
    }
    else {
      resolved.p = []
    }

    return resolved;
    }
};

module.exports = ImageFeaturePoints;
