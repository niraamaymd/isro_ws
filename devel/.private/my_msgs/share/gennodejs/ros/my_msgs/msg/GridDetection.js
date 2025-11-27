// Auto-generated. Do not edit!

// (in-package my_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class GridDetection {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.row1 = null;
      this.row2 = null;
      this.row3 = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('row1')) {
        this.row1 = initObj.row1
      }
      else {
        this.row1 = [];
      }
      if (initObj.hasOwnProperty('row2')) {
        this.row2 = initObj.row2
      }
      else {
        this.row2 = [];
      }
      if (initObj.hasOwnProperty('row3')) {
        this.row3 = initObj.row3
      }
      else {
        this.row3 = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GridDetection
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [row1]
    bufferOffset = _arraySerializer.int32(obj.row1, buffer, bufferOffset, null);
    // Serialize message field [row2]
    bufferOffset = _arraySerializer.int32(obj.row2, buffer, bufferOffset, null);
    // Serialize message field [row3]
    bufferOffset = _arraySerializer.int32(obj.row3, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GridDetection
    let len;
    let data = new GridDetection(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [row1]
    data.row1 = _arrayDeserializer.int32(buffer, bufferOffset, null)
    // Deserialize message field [row2]
    data.row2 = _arrayDeserializer.int32(buffer, bufferOffset, null)
    // Deserialize message field [row3]
    data.row3 = _arrayDeserializer.int32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += 4 * object.row1.length;
    length += 4 * object.row2.length;
    length += 4 * object.row3.length;
    return length + 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'my_msgs/GridDetection';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5913507622e805e6fa0e2a5ae80a4232';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    int32[] row1
    int32[] row2
    int32[] row3
    
    
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GridDetection(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.row1 !== undefined) {
      resolved.row1 = msg.row1;
    }
    else {
      resolved.row1 = []
    }

    if (msg.row2 !== undefined) {
      resolved.row2 = msg.row2;
    }
    else {
      resolved.row2 = []
    }

    if (msg.row3 !== undefined) {
      resolved.row3 = msg.row3;
    }
    else {
      resolved.row3 = []
    }

    return resolved;
    }
};

module.exports = GridDetection;
