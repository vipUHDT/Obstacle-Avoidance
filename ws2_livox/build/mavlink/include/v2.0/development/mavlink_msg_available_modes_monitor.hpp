// MESSAGE AVAILABLE_MODES_MONITOR support class

#pragma once

namespace mavlink {
namespace development {
namespace msg {

/**
 * @brief AVAILABLE_MODES_MONITOR message
 *
 * A change to the sequence number indicates that the set of AVAILABLE_MODES has changed.
        A receiver must re-request all available modes whenever the sequence number changes.
        This is only emitted after the first change and should then be broadcast at low rate (nominally 0.3 Hz) and on change.
      
 */
struct AVAILABLE_MODES_MONITOR : mavlink::Message {
    static constexpr msgid_t MSG_ID = 437;
    static constexpr size_t LENGTH = 1;
    static constexpr size_t MIN_LENGTH = 1;
    static constexpr uint8_t CRC_EXTRA = 30;
    static constexpr auto NAME = "AVAILABLE_MODES_MONITOR";


    uint8_t seq; /*<  Sequence number. The value iterates sequentially whenever AVAILABLE_MODES changes (e.g. support for a new mode is added/removed dynamically). */


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
        ss << "  seq: " << +seq << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << seq;                           // offset: 0
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> seq;                           // offset: 0
    }
};

} // namespace msg
} // namespace development
} // namespace mavlink
