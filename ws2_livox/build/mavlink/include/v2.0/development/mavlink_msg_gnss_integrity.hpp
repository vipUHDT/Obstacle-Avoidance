// MESSAGE GNSS_INTEGRITY support class

#pragma once

namespace mavlink {
namespace development {
namespace msg {

/**
 * @brief GNSS_INTEGRITY message
 *
 * Information about key components of GNSS receivers, like signal authentication, interference and system errors.
 */
struct GNSS_INTEGRITY : mavlink::Message {
    static constexpr msgid_t MSG_ID = 441;
    static constexpr size_t LENGTH = 17;
    static constexpr size_t MIN_LENGTH = 17;
    static constexpr uint8_t CRC_EXTRA = 169;
    static constexpr auto NAME = "GNSS_INTEGRITY";


    uint8_t id; /*<  GNSS receiver id. Must match instance ids of other messages from same receiver. */
    uint32_t system_errors; /*<  Errors in the GPS system. */
    uint8_t authentication_state; /*<  Signal authentication state of the GPS system. */
    uint8_t jamming_state; /*<  Signal jamming state of the GPS system. */
    uint8_t spoofing_state; /*<  Signal spoofing state of the GPS system. */
    uint8_t raim_state; /*<  The state of the RAIM processing. */
    uint16_t raim_hfom; /*< [cm] Horizontal expected accuracy using satellites successfully validated using RAIM. */
    uint16_t raim_vfom; /*< [cm] Vertical expected accuracy using satellites successfully validated using RAIM. */
    uint8_t corrections_quality; /*<  An abstract value representing the estimated quality of incoming corrections, or 255 if not available. */
    uint8_t system_status_summary; /*<  An abstract value representing the overall status of the receiver, or 255 if not available. */
    uint8_t gnss_signal_quality; /*<  An abstract value representing the quality of incoming GNSS signals, or 255 if not available. */
    uint8_t post_processing_quality; /*<  An abstract value representing the estimated PPK quality, or 255 if not available. */


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
        ss << "  id: " << +id << std::endl;
        ss << "  system_errors: " << system_errors << std::endl;
        ss << "  authentication_state: " << +authentication_state << std::endl;
        ss << "  jamming_state: " << +jamming_state << std::endl;
        ss << "  spoofing_state: " << +spoofing_state << std::endl;
        ss << "  raim_state: " << +raim_state << std::endl;
        ss << "  raim_hfom: " << raim_hfom << std::endl;
        ss << "  raim_vfom: " << raim_vfom << std::endl;
        ss << "  corrections_quality: " << +corrections_quality << std::endl;
        ss << "  system_status_summary: " << +system_status_summary << std::endl;
        ss << "  gnss_signal_quality: " << +gnss_signal_quality << std::endl;
        ss << "  post_processing_quality: " << +post_processing_quality << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << system_errors;                 // offset: 0
        map << raim_hfom;                     // offset: 4
        map << raim_vfom;                     // offset: 6
        map << id;                            // offset: 8
        map << authentication_state;          // offset: 9
        map << jamming_state;                 // offset: 10
        map << spoofing_state;                // offset: 11
        map << raim_state;                    // offset: 12
        map << corrections_quality;           // offset: 13
        map << system_status_summary;         // offset: 14
        map << gnss_signal_quality;           // offset: 15
        map << post_processing_quality;       // offset: 16
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> system_errors;                 // offset: 0
        map >> raim_hfom;                     // offset: 4
        map >> raim_vfom;                     // offset: 6
        map >> id;                            // offset: 8
        map >> authentication_state;          // offset: 9
        map >> jamming_state;                 // offset: 10
        map >> spoofing_state;                // offset: 11
        map >> raim_state;                    // offset: 12
        map >> corrections_quality;           // offset: 13
        map >> system_status_summary;         // offset: 14
        map >> gnss_signal_quality;           // offset: 15
        map >> post_processing_quality;       // offset: 16
    }
};

} // namespace msg
} // namespace development
} // namespace mavlink
