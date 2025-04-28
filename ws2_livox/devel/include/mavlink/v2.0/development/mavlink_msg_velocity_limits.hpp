// MESSAGE VELOCITY_LIMITS support class

#pragma once

namespace mavlink {
namespace development {
namespace msg {

/**
 * @brief VELOCITY_LIMITS message
 *
 * Current limits for horizontal speed, vertical speed and yaw rate, as set by SET_VELOCITY_LIMITS.
 */
struct VELOCITY_LIMITS : mavlink::Message {
    static constexpr msgid_t MSG_ID = 355;
    static constexpr size_t LENGTH = 12;
    static constexpr size_t MIN_LENGTH = 12;
    static constexpr uint8_t CRC_EXTRA = 6;
    static constexpr auto NAME = "VELOCITY_LIMITS";


    float horizontal_speed_limit; /*< [m/s] Limit for horizontal movement in MAV_FRAME_LOCAL_NED. NaN: No limit applied */
    float vertical_speed_limit; /*< [m/s] Limit for vertical movement in MAV_FRAME_LOCAL_NED. NaN: No limit applied */
    float yaw_rate_limit; /*< [rad/s] Limit for vehicle turn rate around its yaw axis. NaN: No limit applied */


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
        ss << "  horizontal_speed_limit: " << horizontal_speed_limit << std::endl;
        ss << "  vertical_speed_limit: " << vertical_speed_limit << std::endl;
        ss << "  yaw_rate_limit: " << yaw_rate_limit << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << horizontal_speed_limit;        // offset: 0
        map << vertical_speed_limit;          // offset: 4
        map << yaw_rate_limit;                // offset: 8
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> horizontal_speed_limit;        // offset: 0
        map >> vertical_speed_limit;          // offset: 4
        map >> yaw_rate_limit;                // offset: 8
    }
};

} // namespace msg
} // namespace development
} // namespace mavlink
