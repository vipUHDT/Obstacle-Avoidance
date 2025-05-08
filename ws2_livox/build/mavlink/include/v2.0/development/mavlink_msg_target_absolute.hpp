// MESSAGE TARGET_ABSOLUTE support class

#pragma once

namespace mavlink {
namespace development {
namespace msg {

/**
 * @brief TARGET_ABSOLUTE message
 *
 * Current motion information from sensors on a target
 */
struct TARGET_ABSOLUTE : mavlink::Message {
    static constexpr msgid_t MSG_ID = 510;
    static constexpr size_t LENGTH = 106;
    static constexpr size_t MIN_LENGTH = 106;
    static constexpr uint8_t CRC_EXTRA = 245;
    static constexpr auto NAME = "TARGET_ABSOLUTE";


    uint64_t timestamp; /*< [us] Timestamp (UNIX epoch time). */
    uint8_t id; /*<  The ID of the target if multiple targets are present */
    uint8_t sensor_capabilities; /*<  Bitmap to indicate the sensor's reporting capabilities */
    int32_t lat; /*< [degE7] Target's latitude (WGS84) */
    int32_t lon; /*< [degE7] Target's longitude (WGS84) */
    float alt; /*< [m] Target's altitude (AMSL) */
    std::array<float, 3> vel; /*< [m/s] Target's velocity in its body frame */
    std::array<float, 3> acc; /*< [m/s/s] Linear target's acceleration in its body frame */
    std::array<float, 4> q_target; /*<  Quaternion of the target's orientation from its body frame to the vehicle's NED frame. */
    std::array<float, 3> rates; /*< [rad/s] Target's roll, pitch and yaw rates */
    std::array<float, 2> position_std; /*< [m] Standard deviation of horizontal (eph) and vertical (epv) position errors */
    std::array<float, 3> vel_std; /*< [m/s] Standard deviation of the target's velocity in its body frame */
    std::array<float, 3> acc_std; /*< [m/s/s] Standard deviation of the target's acceleration in its body frame */


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
        ss << "  sensor_capabilities: " << +sensor_capabilities << std::endl;
        ss << "  lat: " << lat << std::endl;
        ss << "  lon: " << lon << std::endl;
        ss << "  alt: " << alt << std::endl;
        ss << "  vel: [" << to_string(vel) << "]" << std::endl;
        ss << "  acc: [" << to_string(acc) << "]" << std::endl;
        ss << "  q_target: [" << to_string(q_target) << "]" << std::endl;
        ss << "  rates: [" << to_string(rates) << "]" << std::endl;
        ss << "  position_std: [" << to_string(position_std) << "]" << std::endl;
        ss << "  vel_std: [" << to_string(vel_std) << "]" << std::endl;
        ss << "  acc_std: [" << to_string(acc_std) << "]" << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << timestamp;                     // offset: 0
        map << lat;                           // offset: 8
        map << lon;                           // offset: 12
        map << alt;                           // offset: 16
        map << vel;                           // offset: 20
        map << acc;                           // offset: 32
        map << q_target;                      // offset: 44
        map << rates;                         // offset: 60
        map << position_std;                  // offset: 72
        map << vel_std;                       // offset: 80
        map << acc_std;                       // offset: 92
        map << id;                            // offset: 104
        map << sensor_capabilities;           // offset: 105
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> timestamp;                     // offset: 0
        map >> lat;                           // offset: 8
        map >> lon;                           // offset: 12
        map >> alt;                           // offset: 16
        map >> vel;                           // offset: 20
        map >> acc;                           // offset: 32
        map >> q_target;                      // offset: 44
        map >> rates;                         // offset: 60
        map >> position_std;                  // offset: 72
        map >> vel_std;                       // offset: 80
        map >> acc_std;                       // offset: 92
        map >> id;                            // offset: 104
        map >> sensor_capabilities;           // offset: 105
    }
};

} // namespace msg
} // namespace development
} // namespace mavlink
