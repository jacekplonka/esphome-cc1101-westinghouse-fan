#pragma once
#include "esphome.h"
#include <CC1101_ESP_Arduino.h>
#include <initializer_list>

namespace esphome::cc1101_fan_controller {

// ============================================================
// Fan RF Protocol Constants
// 29-bit OOK protocol: 20-bit device address + 9-bit command
// ============================================================
static const uint16_t FAN_CMD_SPEED1 = 0x1E8;  // 111101000
static const uint16_t FAN_CMD_SPEED2 = 0x1C8;  // 111001000
static const uint16_t FAN_CMD_SPEED3 = 0x1A9;  // 110101001
static const uint16_t FAN_CMD_SPEED4 = 0x189;  // 110001001
static const uint16_t FAN_CMD_SPEED5 = 0x16A;  // 101101010
static const uint16_t FAN_CMD_SPEED6 = 0x14A;  // 101001010
static const uint16_t FAN_CMD_TOGGLE = 0x191;  // 110010001
static const uint16_t FAN_CMD_LIGHT  = 0x0B5;  // 010110101
static const uint16_t FAN_CMD_INVERT = 0x12B;  // 100101011
static const uint16_t FAN_CMD_FASTER_PREFIX = 0x0C0;
static const uint8_t FAN_CMD_FASTER_STEP = 2;
static const uint16_t FAN_CMD_SLOWER_PREFIX = 0x020;
static const uint8_t FAN_CMD_SLOWER_STEP = 2;

// OOK timings (microseconds), estimated from the recorded 0x093 frames.
// Receiver captures show asymmetric mark/space intervals, so keep each
// symbol's mark and space duration separate rather than using complements.
static const int OOK_MARK_0_US  = 275;
static const int OOK_SPACE_0_US = 725;
static const int OOK_MARK_1_US  = 750;
static const int OOK_SPACE_1_US = 250;
static const int OOK_GAP_US     = 7580;
static const int OOK_TX_REPS    = 3;
static const int OOK_MSG_BITS = 29;  // 20-bit address + 9-bit command
// Captures show 33 symbols: 29 address/command bits, 3 zero trailer bits,
// then one trailing parity symbol. The captured 0x0B3 frame has even parity.

// ============================================================
// Hardware pin defaults — must match your wiring
// ============================================================
#ifndef CC1101_CS_PIN
#define CC1101_CS_PIN    5
#endif
#ifndef CC1101_GDO0_PIN
#define CC1101_GDO0_PIN  25
#endif
#ifndef CC1101_GDO2_PIN
#define CC1101_GDO2_PIN  26
#endif
#ifndef CC1101_SCK_PIN
#define CC1101_SCK_PIN   18
#endif
#ifndef CC1101_MISO_PIN
#define CC1101_MISO_PIN  19
#endif
#ifndef CC1101_MOSI_PIN
#define CC1101_MOSI_PIN  23
#endif
#ifndef FAN_ADDRESS_1
#define FAN_ADDRESS_1 0x94120
#endif
#ifndef FAN_ADDRESS_2
#define FAN_ADDRESS_2 0x64120
#endif

// Forward declaration for ISR
static void rf_record_isr();

class CC1101FanController : public Component {
    static constexpr uint8_t FAN_COUNT = 2;
    CC1101 radio_;
    int gdo0_;
    int gdo2_;
    bool ready_ = false;
    bool listening_ = false;
    bool log_next_capture_ = false;
    bool configuration_initialized_ = false;
    portMUX_TYPE rf_mux_ = portMUX_INITIALIZER_UNLOCKED;

    struct SequencePool {
        uint16_t prefix;
        uint8_t step;
        int excluded_nibble;
    } pools_[6] = {};

    struct FanState {
        uint32_t address;
        int next_nibble[6];
        int power_state;
        int speed_state;
        int light_state;
    } fans_[FAN_COUNT] = {};

    esphome::fan::Fan *fan_entities_[FAN_COUNT] = {};
    esphome::binary_sensor::BinarySensor *light_entities_[FAN_COUNT] = {};
    esphome::text_sensor::TextSensor *last_remote_text_ = nullptr;
    esphome::text_sensor::TextSensor *last_command_text_ = nullptr;
    esphome::text_sensor::TextSensor *discovered_address_text_ = nullptr;
    esphome::switch_::Switch *discovery_switch_ = nullptr;
    static constexpr size_t PERSISTED_VALUE_COUNT = 22;
    int *persisted_values_[PERSISTED_VALUE_COUNT] = {};
    bool persisted_values_bound_ = false;
    bool syncing_fan_state_ = false;
    bool syncing_discovery_switch_ = false;

    int last_remote_address_ = -1;
    int last_remote_command_ = -1;
    int last_fan_command_ = -1;
    int discovered_address_ = -1;
    bool state_dirty_ = false;
    bool discovery_mode_ = false;
    bool repeated_send_active_ = false;
    uint32_t repeated_send_address_ = 0;
    uint16_t repeated_send_command_ = 0;
    uint32_t repeated_send_until_ms_ = 0;
    uint32_t repeated_send_next_ms_ = 0;
    static constexpr uint8_t MAX_BURST_COMMANDS = 8;
    bool burst_send_active_ = false;
    uint32_t burst_send_address_ = 0;
    uint16_t burst_commands_[MAX_BURST_COMMANDS] = {};
    uint8_t burst_command_count_ = 0;
    uint8_t burst_command_index_ = 0;
    uint32_t burst_send_next_ms_ = 0;
    int burst_final_speed_ = -1;
    uint8_t burst_fan_index_ = 0;
    uint32_t last_observed_message_ = 0;
    uint32_t last_observed_ms_ = 0;

public:
    static constexpr int MAX_CHANGES = 512;
    static constexpr int MIN_CAPTURE_EDGES = 55;
    static volatile int rec_timings_[MAX_CHANGES];
    static volatile int rec_levels_[MAX_CHANGES];
    static volatile int rec_initial_level_;
    static volatile int rec_count_;
    static volatile long rec_last_us_;
    static volatile int rec_gdo2_pin_;
    static volatile bool rec_signal_detected_;
    static volatile bool rec_active_;

    CC1101FanController(int sck, int miso, int mosi, int cs, int gdo0, int gdo2)
        : radio_(sck, miso, mosi, cs, gdo0, gdo2), gdo0_(gdo0), gdo2_(gdo2) {
        rec_gdo2_pin_ = gdo2;
        ensure_configuration_();
    }

    void set_fan_addresses(uint32_t fan_1, uint32_t fan_2) {
        if (fan_1 == 0 || fan_1 > 0xFFFFF || fan_2 == 0 || fan_2 > 0xFFFFF || fan_1 == fan_2) {
            ESP_LOGE("cc1101", "Fan addresses must be distinct 20-bit values greater than zero");
            return;
        }
        ensure_configuration_();
        fans_[0].address = fan_1;
        fans_[1].address = fan_2;
    }

    void setup() override {
        if (persisted_values_bound_) {
            restore_state(
                *persisted_values_[0], *persisted_values_[1], *persisted_values_[2],
                *persisted_values_[3], *persisted_values_[4], *persisted_values_[5],
                *persisted_values_[6], *persisted_values_[7], *persisted_values_[8],
                *persisted_values_[9], *persisted_values_[10], *persisted_values_[11],
                *persisted_values_[12], *persisted_values_[13], *persisted_values_[14],
                *persisted_values_[15], *persisted_values_[16], *persisted_values_[17],
                *persisted_values_[18], *persisted_values_[19], *persisted_values_[20],
                *persisted_values_[21]);
        }
        state_dirty_ = true;
        begin();
    }

    float get_setup_priority() const override { return setup_priority::DATA; }

    void bind_entities(esphome::fan::Fan *fan_1, esphome::fan::Fan *fan_2,
                       esphome::binary_sensor::BinarySensor *light_1,
                       esphome::binary_sensor::BinarySensor *light_2,
                       esphome::text_sensor::TextSensor *last_remote_text,
                       esphome::text_sensor::TextSensor *last_command_text,
                       esphome::text_sensor::TextSensor *discovered_address_text,
                       esphome::switch_::Switch *discovery_switch) {
        fan_entities_[0] = fan_1;
        fan_entities_[1] = fan_2;
        for (uint8_t fan = 0; fan < FAN_COUNT; fan++) {
            if (fan_entities_[fan] != nullptr) {
                fan_entities_[fan]->add_on_state_callback([this, fan]() {
                    this->handle_fan_entity_state_(fan);
                });
            }
        }
        light_entities_[0] = light_1;
        light_entities_[1] = light_2;
        last_remote_text_ = last_remote_text;
        last_command_text_ = last_command_text;
        discovered_address_text_ = discovered_address_text;
        discovery_switch_ = discovery_switch;
        if (discovery_switch_ != nullptr) {
            discovery_switch_->add_on_state_callback([this](bool state) {
                if (!this->syncing_discovery_switch_) this->set_discovery_mode(state);
            });
        }
        state_dirty_ = true;
    }

    void bind_button(uint8_t fan, uint8_t action, esphome::button::Button *button) {
        if (fan >= FAN_COUNT || action >= 7 || button == nullptr) return;
        button->add_on_press_callback([this, fan, action]() {
            this->execute_button_(fan, action);
        });
    }

    void bind_buttons(std::initializer_list<esphome::button::Button *> fan_1_buttons,
                      std::initializer_list<esphome::button::Button *> fan_2_buttons) {
        if (fan_1_buttons.size() != 7 || fan_2_buttons.size() != 7)
            ESP_LOGE("cc1101", "Each fan must have seven button entities in the configured action order");
        bind_button_group_(fan_1_buttons, 0);
        bind_button_group_(fan_2_buttons, 1);
    }

    // Keep the YAML globals as ESPHome's persistent storage, while handling
    // their restore and update entirely inside this controller.
    void bind_persistent_values(std::initializer_list<int *> values) {
        if (values.size() != PERSISTED_VALUE_COUNT) {
            ESP_LOGE("cc1101", "Expected %u persistent state values, received %u",
                     (unsigned)PERSISTED_VALUE_COUNT, (unsigned)values.size());
            return;
        }
        size_t index = 0;
        for (int *value : values) persisted_values_[index++] = value;

        persisted_values_bound_ = true;
    }

    // Load ESPHome's restored globals into local state. The controller also
    // writes changes back when it publishes an updated fan state.
    void restore_state(int dimmer, int brighter, int color_temperature,
                       int faster, int slower, int light_toggle,
                       int dimmer_2, int brighter_2, int color_temperature_2,
                       int faster_2, int slower_2, int light_toggle_2,
                       int last_address, int last_command, int last_fan_command,
                       int discovered_address,
                       int fan_power, int fan_speed, int light,
                       int fan_power_2, int fan_speed_2, int light_2) {
        ensure_configuration_();
        pools_[0] = {0x150, 2, -1};
        pools_[1] = {0x100, 2, -1};
        pools_[2] = {0x0B0, 4, 7};
        pools_[3] = {FAN_CMD_FASTER_PREFIX, FAN_CMD_FASTER_STEP, -1};
        pools_[4] = {FAN_CMD_SLOWER_PREFIX, FAN_CMD_SLOWER_STEP, -1};
        pools_[5] = {0x160, 2, -1};
        fans_[0].next_nibble[0] = dimmer;
        fans_[0].next_nibble[1] = brighter;
        fans_[0].next_nibble[2] = color_temperature;
        fans_[0].next_nibble[3] = faster;
        fans_[0].next_nibble[4] = slower;
        fans_[0].next_nibble[5] = light_toggle;
        fans_[1].next_nibble[0] = dimmer_2;
        fans_[1].next_nibble[1] = brighter_2;
        fans_[1].next_nibble[2] = color_temperature_2;
        fans_[1].next_nibble[3] = faster_2;
        fans_[1].next_nibble[4] = slower_2;
        fans_[1].next_nibble[5] = light_toggle_2;
        fans_[0].power_state = fan_power;
        fans_[0].speed_state = fan_speed;
        fans_[0].light_state = light;
        fans_[1].power_state = fan_power_2;
        fans_[1].speed_state = fan_speed_2;
        fans_[1].light_state = light_2;
        last_remote_address_ = last_address;
        last_remote_command_ = last_command;
        last_fan_command_ = last_fan_command;
        discovered_address_ = discovered_address;
        state_dirty_ = false;
    }

    void set_discovery_mode(bool enabled) {
        discovery_mode_ = enabled;
        state_dirty_ = true;
        ESP_LOGI("cc1101_discovery", enabled ? "address discovery enabled" : "address discovery disabled");
    }

    // Home Assistant-facing operations. RF codes and command-pool details stay
    // in this header; the YAML only selects which configured fan is controlled.
    void fan_on(uint8_t fan) { send_fixed_command_(fan, 0x020); }
    void fan_off(uint8_t fan) { send_fixed_command_(fan, 0x093); }
    void fan_light_toggle(uint8_t fan) { send_pool_command_(fan, 5); }
    void fan_color_temperature(uint8_t fan) { send_pool_command_(fan, 2); }
    void fan_direction_toggle(uint8_t fan) { send_fixed_command_(fan, 0x0E4); }
    void fan_dimmer(uint8_t fan) { send_pool_command_(fan, 0); }
    void fan_brighter(uint8_t fan) { send_pool_command_(fan, 1); }
    void fan_max_brightness(uint8_t fan) { repeat_pool_command_(fan, 1, 7000); }
    void fan_min_brightness(uint8_t fan) { repeat_pool_command_(fan, 0, 7000); }
    void fan_speed_up(uint8_t fan) { send_pool_command_(fan, 3); }
    void fan_speed_down(uint8_t fan) { send_pool_command_(fan, 4); }
    void fan_set_speed(uint8_t fan, int target_speed) {
        ensure_configuration_();
        if (fan >= FAN_COUNT || target_speed < 1) return;
        if (target_speed > 6) target_speed = 6;

        if (fans_[fan].power_state == 0) send_fixed_command_(fan, 0x020);
        int current_speed = fans_[fan].speed_state;
        if (current_speed < 1) current_speed = 1;
        while (current_speed < target_speed) {
            send_pool_command_(fan, 3);
            current_speed++;
        }
        while (current_speed > target_speed) {
            send_pool_command_(fan, 4);
            current_speed--;
        }
        fans_[fan].power_state = 1;
        fans_[fan].speed_state = target_speed;
        state_dirty_ = true;
    }
    void fan_max_speed(uint8_t fan) { burst_pool_commands_(fan, 3, 5); }
    void fan_min_speed(uint8_t fan) { burst_pool_commands_(fan, 4, 5); }

    bool begin() {
        ensure_configuration_();
        radio_.init();
        radio_.setMHZ(433.92);
        radio_.setTXPwr(TX_PLUS_10_DBM);
        radio_.setDataRate(2400);
        radio_.setRxBW(RX_BW_162_KHZ);
        radio_.setModulation(ASK_OOK);

        uint8_t partnum = radio_.getPartnum();
        uint8_t version = radio_.getVersion();

        if (version == 0x00 || version == 0xFF) {
            ESP_LOGE("cc1101", "CC1101 not detected (ver=0x%02X). Check wiring.", version);
            return false;
        }

        ready_ = true;
        ESP_LOGI("cc1101", "CC1101 OK  part=0x%02X  ver=0x%02X  433.92 MHz ASK/OOK", partnum, version);
        start_listening_();
        ESP_LOGI("cc1101", "Continuous RX ready: fan 0x%05lX, fan 0x%05lX",
                 (unsigned long)fans_[0].address, (unsigned long)fans_[1].address);
        return true;
    }

    // Call regularly from an ESPHome interval. A quiet period closes each
    // repeated RF transmission, then the captured frames are decoded before
    // the receiver is armed for the next one.
    void loop() override {
        publish_state_updates_();
        if (!ready_ || !listening_) return;
        if (service_scheduled_transmissions_()) {
            publish_state_updates_();
            return;
        }

        const int count = rec_count_;
        if (count <= 0) return;
        const uint32_t now = (uint32_t)micros();
        const uint32_t last = (uint32_t)rec_last_us_;
        const bool full = count >= MAX_CHANGES - 2;
        if (!full && (uint32_t)(now - last) < 25000U) return;

        // The CC1101 OOK output can chatter between packets. Discard short
        // noise bursts in place instead of restarting the receiver for each.
        if (!full && count < MIN_CAPTURE_EDGES) {
            rec_count_ = 0;
            rec_signal_detected_ = false;
            rec_last_us_ = now;
            return;
        }

        stop_listening_();
        const int captured = rec_count_;
        if (captured >= MIN_CAPTURE_EDGES) process_received_signal_(captured);
        start_listening_();
        publish_state_updates_();
    }

    // Keep the existing diagnostic button useful without interrupting the
    // permanent listener or blocking the ESPHome main loop.
    void record_signal() {
        log_next_capture_ = true;
        ESP_LOGI("cc1101", "Continuous RX is active; the next received transmission will be logged.");
    }

    void send_command(uint32_t address, uint16_t command, bool odd_parity = false, bool quiet = false) {
        if (!quiet) {
            repeated_send_active_ = false;
            burst_send_active_ = false;
        }
        if (!ready_) {
            ESP_LOGW("cc1101", "Not initialised — skipping TX");
            return;
        }
        if (address == 0 || address > 0xFFFFF) {
            ESP_LOGE("cc1101", "Refusing TX with invalid fan address 0x%08lX",
                     (unsigned long)address);
            return;
        }

        uint32_t msg = ((address & 0xFFFFF) << 9) | (command & 0x1FF);

        // The captures contain three zero symbols after the 29-bit payload.
        int ones = __builtin_popcount(msg);
        bool parity = odd_parity ? !(ones & 1) : (ones & 1);
        char tx_bits[34];
        for (int bit = 28; bit >= 0; bit--)
            tx_bits[28 - bit] = (msg & (1UL << bit)) ? '1' : '0';
        tx_bits[29] = '0';
        tx_bits[30] = '0';
        tx_bits[31] = '0';
        tx_bits[32] = parity ? '1' : '0';
        tx_bits[33] = '\0';
        if (!quiet) {
            ESP_LOGI("cc1101_tx", "addr=0x%05lX cmd=0x%03X frame33=%s parity=%s reps=%d gap=%dus",
                     (unsigned long)address, command, tx_bits, odd_parity ? "odd" : "even", OOK_TX_REPS, OOK_GAP_US);
            ESP_LOGI("cc1101_tx", "TX timing: bit1 mark/space=%d/%dus; bit0=%d/%dus",
                     OOK_MARK_1_US, OOK_SPACE_1_US, OOK_MARK_0_US, OOK_SPACE_0_US);
        }

        stop_listening_();
        radio_.setTx();
        pinMode(gdo0_, OUTPUT);
        delayMicroseconds(500);

        // Preamble: alternating pulses to wake the receiver's AGC
        for (int i = 0; i < 12; i++) {
            digitalWrite(gdo0_, HIGH);
            delayMicroseconds(OOK_MARK_1_US);
            digitalWrite(gdo0_, LOW);
            delayMicroseconds(OOK_SPACE_1_US);
        }
        delayMicroseconds(OOK_GAP_US);

        for (int rep = 0; rep < OOK_TX_REPS; rep++) {
            portENTER_CRITICAL(&rf_mux_);

            for (int b = OOK_MSG_BITS - 1; b >= 0; b--) {
                bool one = msg & (1UL << b);
                digitalWrite(gdo0_, HIGH);
                delayMicroseconds(one ? OOK_MARK_1_US : OOK_MARK_0_US);
                digitalWrite(gdo0_, LOW);
                delayMicroseconds(one ? OOK_SPACE_1_US : OOK_SPACE_0_US);
            }
            // Three captured zero trailer symbols follow the payload.
            for (int trailer_bit = 0; trailer_bit < 3; trailer_bit++) {
                digitalWrite(gdo0_, HIGH);
                delayMicroseconds(OOK_MARK_0_US);
                digitalWrite(gdo0_, LOW);
                delayMicroseconds(OOK_SPACE_0_US);
            }
            // Final parity symbol; default is even parity based on captured 0x0B3.
            digitalWrite(gdo0_, HIGH);
            delayMicroseconds(parity ? OOK_MARK_1_US : OOK_MARK_0_US);
            digitalWrite(gdo0_, LOW);

            portEXIT_CRITICAL(&rf_mux_);

            delayMicroseconds(OOK_GAP_US);
        }

        digitalWrite(gdo0_, LOW);
        radio_.setIdle();
        if (!quiet)
            ESP_LOGD("cc1101", "TX  addr=0x%05lX  cmd=0x%03X",
                     (unsigned long)address, (unsigned int)command);
        observe_command_(address, command, false);
        start_listening_();
    }

    // Reuse one code for the whole hold, while advancing the pool cursor only
    // once when that code is first transmitted.
    void repeat_cyclic_command_for(uint32_t address, uint16_t command_prefix,
                                   int &next_nibble, uint8_t step,
                                   uint32_t duration_ms, int excluded_nibble = -1) {
        if (!ready_) {
            ESP_LOGW("cc1101", "Not initialised — skipping repeated TX");
            return;
        }
        uint8_t nibble = next_nibble & 0x0F;
        if (excluded_nibble >= 0 && nibble == (excluded_nibble & 0x0F))
            nibble = (nibble + step) & 0x0F;
        const uint16_t command = (command_prefix & 0x1F0) | nibble;
        ESP_LOGI("cc1101_seq", "holding addr=0x%05lX cmd=0x%03X for %lu ms",
                 (unsigned long)address, command, (unsigned long)duration_ms);

        repeated_send_active_ = true;
        burst_send_active_ = false;
        repeated_send_address_ = address;
        repeated_send_command_ = command;
        send_command(address, command, false, true);
        repeated_send_until_ms_ = millis() + duration_ms;
        repeated_send_next_ms_ = millis() + 150U;
    }

    // Send a short run of distinct codes from one command pool. Each code is
    // transmitted once as a three-frame RF packet; duplicate pool values are
    // rejected before the burst starts.
    void send_cyclic_burst(uint32_t address, uint16_t command_prefix,
                           int &next_nibble, uint8_t step, uint8_t command_count,
                           int excluded_nibble = -1) {
        if (!ready_) {
            ESP_LOGW("cc1101", "Not initialised — skipping command burst");
            return;
        }
        if (command_count == 0 || command_count > MAX_BURST_COMMANDS) {
            ESP_LOGW("cc1101", "Invalid burst length: %u", command_count);
            return;
        }

        uint8_t nibble = next_nibble & 0x0F;
        burst_command_count_ = 0;
        for (uint8_t i = 0; i < command_count; i++) {
            if (excluded_nibble >= 0 && nibble == (excluded_nibble & 0x0F))
                nibble = (nibble + step) & 0x0F;
            const uint16_t command = (command_prefix & 0x1F0) | nibble;
            bool duplicate = false;
            for (uint8_t previous = 0; previous < burst_command_count_; previous++)
                if (burst_commands_[previous] == command) duplicate = true;
            if (duplicate) break;
            burst_commands_[burst_command_count_++] = command;
            nibble = (nibble + step) & 0x0F;
        }
        if (burst_command_count_ == 0) {
            ESP_LOGW("cc1101", "Command pool produced no distinct codes");
            return;
        }
        if (burst_command_count_ != command_count)
            ESP_LOGW("cc1101", "Pool has only %u distinct codes; sending that many", burst_command_count_);

        repeated_send_active_ = false;
        burst_send_active_ = true;
        burst_send_address_ = address;
        burst_command_index_ = 0;
        burst_fan_index_ = (uint8_t)find_fan_index_(address);
        burst_final_speed_ = command_prefix == FAN_CMD_FASTER_PREFIX ? 6 :
                             command_prefix == FAN_CMD_SLOWER_PREFIX ? 1 : -1;
        ESP_LOGI("cc1101_seq", "sending %u distinct codes from prefix=0x%03X",
                 burst_command_count_, command_prefix);
        send_next_burst_command_();
    }

    void send_cyclic_command(uint32_t address, uint16_t command_prefix, int &next_nibble, uint8_t step, int excluded_nibble = -1) {
        uint8_t nibble = next_nibble & 0x0F;
        if (excluded_nibble >= 0 && nibble == (excluded_nibble & 0x0F))
            nibble = (nibble + step) & 0x0F;
        const uint16_t command = (command_prefix & 0x1F0) | nibble;
        ESP_LOGI("cc1101_seq", "addr=0x%05lX prefix=0x%03X nibble=0x%X step=+%u",
                 (unsigned long)address, command_prefix, nibble, step);
        if (!ready_) {
            send_command(address, command);
            return;
        }
        send_command(address, command);
        uint8_t next = (nibble + step) & 0x0F;
        if (excluded_nibble >= 0 && next == (excluded_nibble & 0x0F))
            next = (next + step) & 0x0F;
        next_nibble = next;
    }
private:
    void bind_button_group_(std::initializer_list<esphome::button::Button *> buttons,
                            uint8_t fan) {
        uint8_t action = 0;
        for (esphome::button::Button *button : buttons) {
            if (action >= 7) break;
            const uint8_t action_index = action++;
            if (button == nullptr) continue;
            button->add_on_press_callback([this, fan, action_index]() {
                this->execute_button_(fan, action_index);
            });
        }
    }

    void execute_button_(uint8_t fan, uint8_t action) {
        switch (action) {
            case 0: fan_light_toggle(fan); break;
            case 1: fan_color_temperature(fan); break;
            case 2: fan_direction_toggle(fan); break;
            case 3: fan_dimmer(fan); break;
            case 4: fan_brighter(fan); break;
            case 5: fan_max_brightness(fan); break;
            case 6: fan_min_brightness(fan); break;
        }
    }

    void handle_fan_entity_state_(uint8_t fan) {
        if (syncing_fan_state_ || fan >= FAN_COUNT || fan_entities_[fan] == nullptr) return;
        esphome::fan::Fan *entity = fan_entities_[fan];
        if (!entity->state) {
            if (fans_[fan].power_state > 0) fan_off(fan);
            return;
        }

        if (fans_[fan].power_state <= 0) fan_on(fan);
        const int requested_speed = entity->speed;
        if (requested_speed >= 1 && requested_speed != fans_[fan].speed_state)
            fan_set_speed(fan, requested_speed);
    }

    void save_persisted_values_() {
        if (!persisted_values_bound_) return;
        size_t index = 0;
        for (uint8_t fan = 0; fan < FAN_COUNT; fan++)
            for (uint8_t pool = 0; pool < 6; pool++)
                *persisted_values_[index++] = fans_[fan].next_nibble[pool];
        *persisted_values_[index++] = last_remote_address_;
        *persisted_values_[index++] = last_remote_command_;
        *persisted_values_[index++] = last_fan_command_;
        *persisted_values_[index++] = discovered_address_;
        for (uint8_t fan = 0; fan < FAN_COUNT; fan++) {
            *persisted_values_[index++] = fans_[fan].power_state;
            *persisted_values_[index++] = fans_[fan].speed_state;
            *persisted_values_[index++] = fans_[fan].light_state;
        }
    }

    void publish_state_updates_() {
        if (!state_dirty_) return;
        state_dirty_ = false;
        save_persisted_values_();

        for (uint8_t fan = 0; fan < FAN_COUNT; fan++) {
            const bool fan_on = fans_[fan].power_state > 0;
            const int speed = fan_on ? fans_[fan].speed_state : 0;
            esphome::fan::Fan *entity = fan_entities_[fan];
            if (entity != nullptr &&
                (!entity->has_state() || entity->state != fan_on || entity->speed != speed)) {
                syncing_fan_state_ = true;
                entity->state = fan_on;
                entity->speed = speed;
                entity->publish_state();
                syncing_fan_state_ = false;
            }

            esphome::binary_sensor::BinarySensor *light = light_entities_[fan];
            const bool light_on = fans_[fan].light_state > 0;
            if (light != nullptr && (!light->has_state() || light->state != light_on))
                light->publish_state(light_on);
        }

        if (last_remote_text_ != nullptr) {
            char value[32];
            if (last_remote_command_ < 0) {
                snprintf(value, sizeof(value), "No remote command seen");
            } else {
                snprintf(value, sizeof(value), "0x%05X / 0x%03X",
                         (unsigned int)last_remote_address_,
                         (unsigned int)last_remote_command_);
            }
            if (!last_remote_text_->has_state() || last_remote_text_->get_state() != value)
                last_remote_text_->publish_state(value);
        }

        if (last_command_text_ != nullptr) {
            char value[32];
            if (last_fan_command_ < 0) {
                snprintf(value, sizeof(value), "No command sent or received");
            } else {
                snprintf(value, sizeof(value), "0x%03X", (unsigned int)last_fan_command_);
            }
            if (!last_command_text_->has_state() || last_command_text_->get_state() != value)
                last_command_text_->publish_state(value);
        }

        if (discovered_address_text_ != nullptr) {
            char value[16];
            if (discovered_address_ < 0) {
                snprintf(value, sizeof(value), "No address found");
            } else {
                snprintf(value, sizeof(value), "0x%05X", (unsigned int)discovered_address_);
            }
            if (!discovered_address_text_->has_state() || discovered_address_text_->get_state() != value)
                discovered_address_text_->publish_state(value);
        }

        if (discovery_switch_ != nullptr &&
            (!discovery_switch_->has_state() || discovery_switch_->state != discovery_mode_)) {
            syncing_discovery_switch_ = true;
            discovery_switch_->publish_state(discovery_mode_);
            syncing_discovery_switch_ = false;
        }
    }

    void ensure_configuration_() {
        if (configuration_initialized_) return;
        pools_[0] = {0x150, 2, -1};
        pools_[1] = {0x100, 2, -1};
        pools_[2] = {0x0B0, 4, 7};
        pools_[3] = {FAN_CMD_FASTER_PREFIX, FAN_CMD_FASTER_STEP, -1};
        pools_[4] = {FAN_CMD_SLOWER_PREFIX, FAN_CMD_SLOWER_STEP, -1};
        pools_[5] = {0x160, 2, -1};
        fans_[0].address = FAN_ADDRESS_1;
        fans_[1].address = FAN_ADDRESS_2;
        const int default_cursors[6] = {0, 1, 3, 6, 0, 5};
        for (uint8_t fan = 0; fan < FAN_COUNT; fan++) {
            for (uint8_t pool = 0; pool < 6; pool++)
                fans_[fan].next_nibble[pool] = default_cursors[pool];
            fans_[fan].power_state = 0;
            fans_[fan].speed_state = 0;
            fans_[fan].light_state = 0;
        }
        configuration_initialized_ = true;
    }

    int find_fan_index_(uint32_t address) const {
        for (uint8_t i = 0; i < FAN_COUNT; i++)
            if (fans_[i].address == address) return i;
        return -1;
    }

    void send_fixed_command_(uint8_t fan, uint16_t command) {
        ensure_configuration_();
        if (fan >= FAN_COUNT) return;
        send_command(fans_[fan].address, command);
    }

    void send_pool_command_(uint8_t fan, uint8_t pool) {
        ensure_configuration_();
        if (fan >= FAN_COUNT || pool >= 6) return;
        SequencePool &sequence = pools_[pool];
        send_cyclic_command(fans_[fan].address, sequence.prefix,
                            fans_[fan].next_nibble[pool], sequence.step,
                            sequence.excluded_nibble);
    }

    void repeat_pool_command_(uint8_t fan, uint8_t pool, uint32_t duration_ms) {
        ensure_configuration_();
        if (fan >= FAN_COUNT || pool >= 6) return;
        SequencePool &sequence = pools_[pool];
        repeat_cyclic_command_for(fans_[fan].address, sequence.prefix,
                                  fans_[fan].next_nibble[pool], sequence.step,
                                  duration_ms, sequence.excluded_nibble);
    }

    void burst_pool_commands_(uint8_t fan, uint8_t pool, uint8_t count) {
        ensure_configuration_();
        if (fan >= FAN_COUNT || pool >= 6) return;
        SequencePool &sequence = pools_[pool];
        send_cyclic_burst(fans_[fan].address, sequence.prefix,
                          fans_[fan].next_nibble[pool], sequence.step, count,
                          sequence.excluded_nibble);
    }

    bool service_scheduled_transmissions_() {
        if (repeated_send_active_) {
            const uint32_t now = millis();
            if ((int32_t)(now - repeated_send_until_ms_) >= 0) {
                repeated_send_active_ = false;
                ESP_LOGI("cc1101_seq", "timed command complete");
                return true;
            }
            if ((int32_t)(now - repeated_send_next_ms_) >= 0) {
                send_command(repeated_send_address_, repeated_send_command_, false, true);
                repeated_send_next_ms_ = millis() + 150U;
                return true;
            }
        }
        if (burst_send_active_ && (int32_t)(millis() - burst_send_next_ms_) >= 0) {
            send_next_burst_command_();
            return true;
        }
        return false;
    }

    void send_next_burst_command_() {
        if (burst_command_index_ >= burst_command_count_) {
            finish_burst_();
            return;
        }
        const uint16_t command = burst_commands_[burst_command_index_++];
        send_command(burst_send_address_, command, false, true);
        if (burst_command_index_ >= burst_command_count_) {
            finish_burst_();
        } else {
            burst_send_next_ms_ = millis() + 150U;
        }
    }

    void finish_burst_() {
        burst_send_active_ = false;
        if (burst_final_speed_ >= 1 && burst_fan_index_ < FAN_COUNT) {
            fans_[burst_fan_index_].power_state = 1;
            fans_[burst_fan_index_].speed_state = burst_final_speed_;
            state_dirty_ = true;
        }
        burst_final_speed_ = -1;
        ESP_LOGI("cc1101_seq", "distinct-code burst complete");
    }

    void start_listening_() {
        if (!ready_ || listening_) return;
        radio_.setRx();
        pinMode(gdo2_, INPUT);
        rec_count_ = 0;
        rec_signal_detected_ = false;
        rec_initial_level_ = digitalRead(gdo2_);
        rec_last_us_ = micros();
        rec_active_ = true;
        attachInterrupt(digitalPinToInterrupt(gdo2_), rf_record_isr, CHANGE);
        listening_ = true;
    }

    void stop_listening_() {
        if (!listening_) return;
        rec_active_ = false;
        detachInterrupt(digitalPinToInterrupt(gdo2_));
        listening_ = false;
    }

    void process_received_signal_(int count) {
        static constexpr int MAX_DISCOVERY_CANDIDATES = 8;
        uint32_t previous_message = 0;
        bool have_previous = false;
        int matched_frames = 0;
        uint32_t discovery_messages[MAX_DISCOVERY_CANDIDATES] = {};
        uint8_t discovery_counts[MAX_DISCOVERY_CANDIDATES] = {};
        int discovery_candidate_count = 0;

        for (int gap = 0; gap < count; gap++) {
            if (rec_timings_[gap] < 5000) continue;
            const int first_mark = gap + 1;
            int end = first_mark;
            while (end < count && rec_timings_[end] < 5000) end++;

            const int intervals = end - first_mark;
            const int symbols = (intervals + 1) / 2;
            if (symbols < OOK_MSG_BITS || symbols > 60) continue;

            uint32_t message = 0;
            bool valid = true;
            for (int bit = 0; bit < OOK_MSG_BITS; bit++) {
                const int mark_index = first_mark + bit * 2;
                if (mark_index >= end) { valid = false; break; }
                message = (message << 1) | (rec_timings_[mark_index] > 500 ? 1U : 0U);
            }
            if (!valid) continue;

            const uint32_t address = (message >> 9) & 0xFFFFF;
            const uint16_t command = message & 0x1FF;
            const int fan_index = find_fan_index_(address);
            if (fan_index < 0) {
                if (!discovery_mode_ || address == 0 || address == 0xFFFFF) continue;
                int candidate = 0;
                while (candidate < discovery_candidate_count &&
                       discovery_messages[candidate] != message) candidate++;
                if (candidate < discovery_candidate_count) {
                    if (discovery_counts[candidate] < 0xFF) discovery_counts[candidate]++;
                } else if (discovery_candidate_count < MAX_DISCOVERY_CANDIDATES) {
                    discovery_messages[discovery_candidate_count] = message;
                    discovery_counts[discovery_candidate_count] = 1;
                    discovery_candidate_count++;
                }
                continue;
            }
            matched_frames++;

            // A remote usually repeats one frame several times per press.
            // Apply the press once so toggle commands do not toggle repeatedly.
            if (have_previous && message == previous_message) continue;
            previous_message = message;
            have_previous = true;
            observe_command_(address, command, true);
            ESP_LOGI("cc1101_rx", "remote addr=0x%05lX cmd=0x%03X", (unsigned long)address, command);

            if (log_next_capture_)
                ESP_LOGI("rf_decode", "matched address=0x%05lX command=0x%03X", (unsigned long)address, command);
        }

        // Ignore one-off noise: accept a new address only after the same
        // address/command frame appears at least twice in the capture.
        if (discovery_mode_) {
            for (int i = 0; i < discovery_candidate_count; i++) {
                if (discovery_counts[i] < 2) continue;
                discovered_address_ = (int)((discovery_messages[i] >> 9) & 0xFFFFF);
                discovery_mode_ = false;
                state_dirty_ = true;
                ESP_LOGI("cc1101_discovery", "found fan address 0x%05X", (unsigned int)discovered_address_);
                break;
            }
        }

        if (log_next_capture_) {
            ESP_LOGI("rf_decode", "capture edges=%d matched_frames=%d", count, matched_frames);
            log_next_capture_ = false;
        }
    }

    void observe_command_(uint32_t address, uint16_t command, bool received) {
        const int fan_index = find_fan_index_(address);
        if (fan_index < 0) return;
        FanState &fan = fans_[fan_index];

        for (uint8_t i = 0; i < 6; i++) {
            const SequencePool &pool = pools_[i];
            if ((command & 0x1F0) != pool.prefix) continue;
            uint8_t next = ((command & 0x0F) + pool.step) & 0x0F;
            if (pool.excluded_nibble >= 0 && next == (pool.excluded_nibble & 0x0F))
                next = (next + pool.step) & 0x0F;
            fan.next_nibble[i] = next;
        }

        last_fan_command_ = command;
        state_dirty_ = true;

        const uint32_t message = (address << 9) | command;
        const uint32_t now = millis();
        const bool repeated = received && message == last_observed_message_ &&
                              (uint32_t)(now - last_observed_ms_) < 250U;
        if (received) {
            last_observed_message_ = message;
            last_observed_ms_ = now;
            last_remote_address_ = (int)address;
            last_remote_command_ = command;
        }
        if (repeated) return;

        if (command == FAN_CMD_SPEED1 || command == FAN_CMD_SPEED2 || command == FAN_CMD_SPEED3 ||
            command == FAN_CMD_SPEED4 || command == FAN_CMD_SPEED5 || command == FAN_CMD_SPEED6) {
            fan.power_state = 1;
            if (command == FAN_CMD_SPEED1) fan.speed_state = 1;
            else if (command == FAN_CMD_SPEED2) fan.speed_state = 2;
            else if (command == FAN_CMD_SPEED3) fan.speed_state = 3;
            else if (command == FAN_CMD_SPEED4) fan.speed_state = 4;
            else if (command == FAN_CMD_SPEED5) fan.speed_state = 5;
            else fan.speed_state = 6;
        } else if (command == 0x093) {
            fan.power_state = 0;
            fan.speed_state = 0;
        } else if (command == 0x020) {
            fan.power_state = 1;
            if (fan.speed_state < 1) fan.speed_state = 1;
        } else if (command == FAN_CMD_TOGGLE) {
            if (fan.power_state >= 0) {
                fan.power_state = !fan.power_state;
                fan.speed_state = fan.power_state ? 1 : 0;
            }
        } else if ((command & 0x1F0) == FAN_CMD_FASTER_PREFIX) {
            fan.power_state = 1;
            if (fan.speed_state < 1) fan.speed_state = 1;
            else if (fan.speed_state < 6) ++fan.speed_state;
        } else if ((command & 0x1F0) == FAN_CMD_SLOWER_PREFIX && command != FAN_CMD_SLOWER_PREFIX) {
            fan.power_state = 1;
            if (fan.speed_state < 1) fan.speed_state = 1;
            else if (fan.speed_state > 1) --fan.speed_state;
        }

        if ((command == FAN_CMD_LIGHT || (command & 0x1F0) == 0x160) &&
            fan.light_state >= 0)
            fan.light_state = !fan.light_state;
    }
};

// Static member definitions
volatile int CC1101FanController::rec_timings_[CC1101FanController::MAX_CHANGES];
volatile int CC1101FanController::rec_levels_[CC1101FanController::MAX_CHANGES];
volatile int CC1101FanController::rec_initial_level_ = 0;
volatile int CC1101FanController::rec_count_ = 0;
volatile long CC1101FanController::rec_last_us_ = 0;
volatile int CC1101FanController::rec_gdo2_pin_ = CC1101_GDO2_PIN;
volatile bool CC1101FanController::rec_signal_detected_ = false;
volatile bool CC1101FanController::rec_active_ = false;

static void IRAM_ATTR rf_record_isr() {
    if (!CC1101FanController::rec_active_ ||
        CC1101FanController::rec_count_ >= CC1101FanController::MAX_CHANGES)
        return;
    long now = micros();
    int index = CC1101FanController::rec_count_;
    CC1101FanController::rec_timings_[index] =
        (int)(now - CC1101FanController::rec_last_us_);
    CC1101FanController::rec_levels_[index] = digitalRead(CC1101FanController::rec_gdo2_pin_);
    CC1101FanController::rec_count_ = index + 1;
    CC1101FanController::rec_last_us_ = now;
    CC1101FanController::rec_signal_detected_ = true;
}

}  // namespace esphome::cc1101_fan_controller
