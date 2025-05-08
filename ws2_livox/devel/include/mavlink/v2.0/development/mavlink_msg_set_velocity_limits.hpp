// MESSAGE SET_VELOCITY_LIMITS support class

#pragma once

namespace mavlink {
namespace development {
namespace msg {

/**
 * @brief SET_VELOCITY_LIMITS message
 *
 * Set temporary maximum limits for horizontal speed, vertical speed and yaw rate.
        The consumer must stream the current limits in VELOCITY_LIMITS at 1 Hz or more (when limits are being set).
        The consumer should latch the limits until a new limit is received or the mode is changed.
      
 */
struct SET_VELOCITY_LIMITS : mavlink::Message {
    static constexpr msgid_t MSG_ID = 354;
    static constexpr size_t LENGTH = 14;
    static constexpr size_t MIN_LENGTH = 14;
    static constexpr uint8_t CRC_EXTRA = 210;
    static constexpr auto NAME = "SET_VELOCITY_LIMITS";


    uint8_t target_system; /*<  System ID (0 for broadcast). */
    uint8_t target_component; /*<  Component ID (0 for broadcast). */
    float horizontal_speed_limit; /*< [m/s] Limit for horizontal movement in MAV_FRAME_LOCAL_NED. NaN: Field not used (ignore) */
    float vertical_speed_limit; /*< [m/s] Limit for vertical movement in MAV_FRAME_LOCAL_NED. NaN: Field not used (ignore) */
    float yaw_rate_limit; /*< [rad/s] Limit for vehicle turn rate around its yaw axis. NaN: Field not used (ignore) */


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
        ss << "  target_system: " << +target_system << std::endl;
        ss << "  target_component: " << +target_component << std::endl;
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
        map << target_system;                 // offset: 12
        map << target_component;              // offset: 13
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> horizontal_speed_limit;        // offset: 0
        map >> vertical_speed_limit;          // offset: 4
        map >> yaw_rate_limit;                // offset: 8
        map >> target_system;                 // offset: 12
        map >> target_component;              // offset: 13
    }
};

} // namespace msg
} // namespace development
} // namespace mavlink
