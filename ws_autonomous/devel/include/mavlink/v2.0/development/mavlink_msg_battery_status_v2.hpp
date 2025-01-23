// MESSAGE BATTERY_STATUS_V2 support class

#pragma once

namespace mavlink {
namespace development {
namespace msg {

/**
 * @brief BATTERY_STATUS_V2 message
 *
 * Battery dynamic information.
        This should be streamed (nominally at 1Hz).
        Static/invariant battery information is sent in BATTERY_INFO.
        Note that smart batteries should set the MAV_BATTERY_STATUS_FLAGS_CAPACITY_RELATIVE_TO_FULL bit to indicate that supplied capacity values are relative to a battery that is known to be full.
        Power monitors would not set this bit, indicating that capacity_consumed is relative to drone power-on, and that other values are estimated based on the assumption that the battery was full on power-on.
      
 */
struct BATTERY_STATUS_V2 : mavlink::Message {
    static constexpr msgid_t MSG_ID = 369;
    static constexpr size_t LENGTH = 24;
    static constexpr size_t MIN_LENGTH = 24;
    static constexpr uint8_t CRC_EXTRA = 151;
    static constexpr auto NAME = "BATTERY_STATUS_V2";


    uint8_t id; /*<  Battery ID */
    int16_t temperature; /*< [cdegC] Temperature of the whole battery pack (not internal electronics). INT16_MAX field not provided. */
    float voltage; /*< [V] Battery voltage (total). NaN: field not provided. */
    float current; /*< [A] Battery current (through all cells/loads). Positive value when discharging and negative if charging. NaN: field not provided. */
    float capacity_consumed; /*< [Ah] Consumed charge. NaN: field not provided. This is either the consumption since power-on or since the battery was full, depending on the value of MAV_BATTERY_STATUS_FLAGS_CAPACITY_RELATIVE_TO_FULL. */
    float capacity_remaining; /*< [Ah] Remaining charge (until empty). NaN: field not provided. Note: If MAV_BATTERY_STATUS_FLAGS_CAPACITY_RELATIVE_TO_FULL is unset, this value is based on the assumption the battery was full when the system was powered. */
    uint8_t percent_remaining; /*< [%] Remaining battery energy. Values: [0-100], UINT8_MAX: field not provided. */
    uint32_t status_flags; /*<  Fault, health, readiness, and other status indications. */


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
        ss << "  temperature: " << temperature << std::endl;
        ss << "  voltage: " << voltage << std::endl;
        ss << "  current: " << current << std::endl;
        ss << "  capacity_consumed: " << capacity_consumed << std::endl;
        ss << "  capacity_remaining: " << capacity_remaining << std::endl;
        ss << "  percent_remaining: " << +percent_remaining << std::endl;
        ss << "  status_flags: " << status_flags << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << voltage;                       // offset: 0
        map << current;                       // offset: 4
        map << capacity_consumed;             // offset: 8
        map << capacity_remaining;            // offset: 12
        map << status_flags;                  // offset: 16
        map << temperature;                   // offset: 20
        map << id;                            // offset: 22
        map << percent_remaining;             // offset: 23
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> voltage;                       // offset: 0
        map >> current;                       // offset: 4
        map >> capacity_consumed;             // offset: 8
        map >> capacity_remaining;            // offset: 12
        map >> status_flags;                  // offset: 16
        map >> temperature;                   // offset: 20
        map >> id;                            // offset: 22
        map >> percent_remaining;             // offset: 23
    }
};

} // namespace msg
} // namespace development
} // namespace mavlink
