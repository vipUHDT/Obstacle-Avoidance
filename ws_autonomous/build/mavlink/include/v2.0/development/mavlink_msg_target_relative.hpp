// MESSAGE TARGET_RELATIVE support class

#pragma once

namespace mavlink {
namespace development {
namespace msg {

/**
 * @brief TARGET_RELATIVE message
 *
 * The location of a target measured by MAV's onboard sensors. 
 */
struct TARGET_RELATIVE : mavlink::Message {
    static constexpr msgid_t MSG_ID = 511;
    static constexpr size_t LENGTH = 71;
    static constexpr size_t MIN_LENGTH = 71;
    static constexpr uint8_t CRC_EXTRA = 28;
    static constexpr auto NAME = "TARGET_RELATIVE";


    uint64_t timestamp; /*< [us] Timestamp (UNIX epoch time) */
    uint8_t id; /*<  The ID of the target if multiple targets are present */
    uint8_t frame; /*<  Coordinate frame used for following fields. */
    float x; /*< [m] X Position of the target in TARGET_OBS_FRAME */
    float y; /*< [m] Y Position of the target in TARGET_OBS_FRAME */
    float z; /*< [m] Z Position of the target in TARGET_OBS_FRAME */
    std::array<float, 3> pos_std; /*< [m] Standard deviation of the target's position in TARGET_OBS_FRAME */
    float yaw_std; /*< [rad] Standard deviation of the target's orientation in TARGET_OBS_FRAME */
    std::array<float, 4> q_target; /*<  Quaternion of the target's orientation from the target's frame to the TARGET_OBS_FRAME (w, x, y, z order, zero-rotation is 1, 0, 0, 0) */
    std::array<float, 4> q_sensor; /*<  Quaternion of the sensor's orientation from TARGET_OBS_FRAME to vehicle-carried NED. (Ignored if set to (0,0,0,0)) (w, x, y, z order, zero-rotation is 1, 0, 0, 0) */
    uint8_t type; /*<  Type of target */


    inline std::string get_name(void) const override
    {
            return NAME;
    }

    inline Info get_message_info(void) const override
    {
            return { MSG_ID, LENGTH, MIN_LENGTH, CRC_EXTRA };
    }

    inline std::string to_yaml(void) const override
    {
        std::stringstream ss;

        ss << NAME << ":" << std::endl;
        ss << "  timestamp: " << timestamp << std::endl;
        ss << "  id: " << +id << std::endl;
        ss << "  frame: " << +frame << std::endl;
        ss << "  x: " << x << std::endl;
        ss << "  y: " << y << std::endl;
        ss << "  z: " << z << std::endl;
        ss << "  pos_std: [" << to_string(pos_std) << "]" << std::endl;
        ss << "  yaw_std: " << yaw_std << std::endl;
        ss << "  q_target: [" << to_string(q_target) << "]" << std::endl;
        ss << "  q_sensor: [" << to_string(q_sensor) << "]" << std::endl;
        ss << "  type: " << +type << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << timestamp;                     // offset: 0
        map << x;                             // offset: 8
        map << y;                             // offset: 12
        map << z;                             // offset: 16
        map << pos_std;                       // offset: 20
        map << yaw_std;                       // offset: 32
        map << q_target;                      // offset: 36
        map << q_sensor;                      // offset: 52
        map << id;                            // offset: 68
        map << frame;                         // offset: 69
        map << type;                          // offset: 70
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> timestamp;                     // offset: 0
        map >> x;                             // offset: 8
        map >> y;                             // offset: 12
        map >> z;                             // offset: 16
        map >> pos_std;                       // offset: 20
        map >> yaw_std;                       // offset: 32
        map >> q_target;                      // offset: 36
        map >> q_sensor;                      // offset: 52
        map >> id;                            // offset: 68
        map >> frame;                         // offset: 69
        map >> type;                          // offset: 70
    }
};

} // namespace msg
} // namespace development
} // namespace mavlink
