#ifndef __COMPONENT_ZEHNDER_H__
#define __COMPONENT_ZEHNDER_H__

#include "esphome/core/component.h"
#include "esphome/core/preferences.h"
#include "esphome/core/hal.h"
#include "esphome/components/spi/spi.h"
#include "esphome/components/fan/fan_state.h"
#include "esphome/components/nrf905/nRF905.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"

#include <vector>
#include <string>
#include <functional>

namespace esphome {
namespace zehnder {

#define FAN_FRAMESIZE 16        // Each frame consists of 16 bytes
#define FAN_TX_FRAMES 4         // Retransmit every transmitted frame 4 times
#define FAN_TX_RETRIES 10       // Retry transmission 10 times if no reply is received
#define FAN_TTL 250             // 0xFA, default time-to-live for a frame
#define FAN_REPLY_TIMEOUT 2000  // Wait 2000ms for receiving a reply

/* Fan device types */
enum {
  FAN_TYPE_BROADCAST = 0x00,       // Broadcast to all devices
  FAN_TYPE_MAIN_UNIT = 0x01,       // Fans
  FAN_TYPE_REMOTE_CONTROL = 0x03,  // Remote controls
  FAN_TYPE_CO2_SENSOR = 0x18
};  // CO2 sensors

/* Fan commands */
enum {
  FAN_FRAME_SETVOLTAGE = 0x01,  // Set speed (voltage / percentage)
  FAN_FRAME_SETSPEED = 0x02,    // Set speed (preset)
  FAN_FRAME_SETTIMER = 0x03,    // Set speed with timer
  FAN_NETWORK_JOIN_REQUEST = 0x04,
  FAN_FRAME_SETSPEED_REPLY = 0x05,
  FAN_NETWORK_JOIN_OPEN = 0x06,
  FAN_TYPE_FAN_SETTINGS = 0x07,  // Current settings, sent by fan in reply to 0x01, 0x02, 0x10
  FAN_FRAME_0B = 0x0B,
  FAN_NETWORK_JOIN_ACK = 0x0C,
  // FAN_NETWORK_JOIN_FINISH = 0x0D,
  FAN_TYPE_QUERY_NETWORK = 0x0D,
  FAN_TYPE_QUERY_DEVICE = 0x10,
  FAN_FRAME_SETVOLTAGE_REPLY = 0x1D,

  // New diagnostic commands (actual command codes would need to be discovered)
  // FAN_TYPE_QUERY_ERROR_STATUS = 0x30,    // Request error codes
  // FAN_TYPE_ERROR_STATUS_RESPONSE = 0x31, // Response with error codes
  // FAN_TYPE_QUERY_FILTER_STATUS = 0x32,   // Request filter status
  // FAN_TYPE_FILTER_STATUS_RESPONSE = 0x33 // Response with filter status
};

/* Fan speed presets */
enum {
  FAN_SPEED_AUTO = 0x00,    // Off:      0% or  0.0 volt
  FAN_SPEED_LOW = 0x01,     // Low:     30% or  3.0 volt
  FAN_SPEED_MEDIUM = 0x02,  // Medium:  50% or  5.0 volt
  FAN_SPEED_HIGH = 0x03,    // High:    90% or  9.0 volt
  FAN_SPEED_MAX = 0x04
};  // Max:    100% or 10.0 volt

#define NETWORK_LINK_ID 0xA55A5AA5
#define NETWORK_DEFAULT_ID 0xE7E7E7E7
#define FAN_JOIN_DEFAULT_TIMEOUT 10000

typedef enum { ResultOk, ResultBusy, ResultFailure } Result;

// --- Struct Definitions --- (Define BEFORE use in RfFrame)

typedef struct __attribute__((packed)) {
  uint8_t speed;
} RfPayloadFanSetSpeed;

typedef struct __attribute__((packed)) {
  uint8_t speed;
  uint8_t timer;
} RfPayloadFanSetTimer;

typedef struct __attribute__((packed)) {
  uint32_t networkId;
} RfPayloadNetworkJoinRequest;

typedef struct __attribute__((packed)) {
  uint32_t networkId;
} RfPayloadNetworkJoinOpen;

typedef struct __attribute__((packed)) {
  uint8_t speed;
  uint8_t voltage;
  uint8_t timer;
} RfPayloadFanSettings;

typedef struct __attribute__((packed)) {
  uint32_t networkId;
} RfPayloadNetworkJoinAck;

// RfFrame Struct using the above payload definitions
typedef struct __attribute__((packed)) {
  uint8_t rx_type;
  uint8_t rx_id;
  uint8_t tx_type;
  uint8_t tx_id;
  uint8_t ttl;
  uint8_t command;
  uint8_t parameter_count;
  union {
    uint8_t parameters[9]; // Max payload size is 9 (16 Frame - 7 Header)
    RfPayloadFanSetSpeed setSpeed;
    RfPayloadFanSetTimer setTimer;
    RfPayloadNetworkJoinRequest networkJoinRequest;
    RfPayloadNetworkJoinOpen networkJoinOpen;
    RfPayloadFanSettings fanSettings;
    RfPayloadNetworkJoinAck networkJoinAck;
    // Add other payloads here if needed
  } payload;
} RfFrame;

// --- ZehnderRF Class Definition ---

class ZehnderRF : public Component, public fan::Fan {
 public:
  ZehnderRF();

  // Standard ESPHome component methods
  void setup() override;
  void dump_config() override;
  void loop() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  // Setup methods
  void set_rf(nrf905::nRF905 *const pRf) { rf_ = pRf; }
  void set_update_interval(const uint32_t interval) { interval_ = interval; }

  // Sensor setters
  void set_ventilation_percentage_sensor(sensor::Sensor *sensor) { ventilation_percentage_sensor_ = sensor; }
  void set_timer_binary_sensor(binary_sensor::BinarySensor *sensor) { timer_binary_sensor_ = sensor; }
  void set_ventilation_mode_text_sensor(text_sensor::TextSensor *sensor) { ventilation_mode_text_sensor_ = sensor; }
  // Keep diagnostic sensor setters even if query logic is removed
  void set_filter_remaining_sensor(sensor::Sensor *sensor) { filter_remaining_sensor_ = sensor; }
  void set_filter_runtime_sensor(sensor::Sensor *sensor) { filter_runtime_sensor_ = sensor; }
  void set_error_count_sensor(sensor::Sensor *sensor) { error_count_sensor_ = sensor; }
  void set_error_code_sensor(text_sensor::TextSensor *sensor) { error_code_sensor_ = sensor; }

  // Fan interface implementation
  fan::FanTraits get_traits() override;
  int get_speed_count() { return this->speed_count_; }
  void control(const fan::FanCall &call) override;

  // Public methods/members for YAML access
  void setSpeed(const uint8_t speed, const uint8_t timer = 0);
  bool timer = false;
  int voltage = 0;

 protected:
  // Core logic methods
  void queryDevice(void);
  uint8_t createDeviceID(void);
  std::string speedToMode_(uint8_t speed_preset);

  // Discovery logic methods
  void discoveryStart(const uint8_t deviceId);
  void handleDiscoveryLinkRequest(const RfFrame *const frame);
  void handleDiscoveryJoinResponse(const RfFrame *const frame);
  void handleDiscoveryJoinComplete(const RfFrame *const frame);

  // Settings handler method
  void handleFanSettings(const RfFrame *const frame);

  // RF Layer interaction methods
  Result startTransmit(const uint8_t *const pData, const int8_t rxRetries = -1,
                       const std::function<void(void)> timeoutCallback = nullptr);
  void rfComplete(void); // Called when TX/RX cycle finishes (success or timeout)
  void rfHandler(void); // Handles timeouts, retries, airway check
  void rfHandleReceived(const uint8_t *const pData, const uint8_t dataLength); // Called by nRF905 callback

  // --- State Machines ---
  typedef enum {
    StateStartup,
    StateStartDiscovery,
    StateDiscoveryWaitForLinkRequest,
    StateDiscoveryWaitForJoinResponse,
    StateDiscoveryJoinComplete,
    StateIdle,
    StateWaitFanSettings, // State for waiting for reply to queryDevice (0x10)
    StateWaitSetSpeedConfirm, // State for waiting for TX confirmation of setSpeed/setTimer
    // Removed diagnostic query states
  } State;
  State state_{StateStartup};

  typedef enum {
    RfStateIdle,
    RfStateWaitAirwayFree,
    RfStateTxBusy, // Waiting for TX complete interrupt from nRF905
    RfStateRxWait, // Waiting for RX complete interrupt or timeout
  } RfState;
  RfState rfState_{RfStateIdle};

  // --- Configuration & Preferences ---
  typedef struct {
    uint32_t fan_networkId;
    uint8_t fan_my_device_type;
    uint8_t fan_my_device_id;
    uint8_t fan_main_unit_type;
    uint8_t fan_main_unit_id;
  } Config;
  Config config_;
  ESPPreferenceObject pref_;

  // --- Member Variables ---
  nrf905::nRF905 *rf_ = nullptr;
  uint32_t interval_ = 15000; // Default update interval (ms)
  int speed_count_ = 4; // Default speed count (Auto=0, Low=1, Med=2, High=3, Max=4 -> use 4 speeds for HA)

  // Sensor pointers
  sensor::Sensor *ventilation_percentage_sensor_{nullptr};
  binary_sensor::BinarySensor *timer_binary_sensor_{nullptr};
  text_sensor::TextSensor *ventilation_mode_text_sensor_{nullptr};
  sensor::Sensor *filter_remaining_sensor_{nullptr};
  sensor::Sensor *filter_runtime_sensor_{nullptr};
  sensor::Sensor *error_count_sensor_{nullptr};
  text_sensor::TextSensor *error_code_sensor_{nullptr};

  // RF state tracking
  uint32_t lastFanQuery_{0};
  uint32_t msgSendTime_{0}; // Time when the last message expecting a reply was sent
  uint32_t airwayFreeWaitTime_{0}; // Time when we started waiting for airway clear
  int8_t retries_{-1}; // Retries remaining for the current TX/RX cycle (-1 means no reply expected)
  std::function<void(void)> onReceiveTimeout_ = nullptr; // Callback on timeout

  // Pending command state
  uint8_t newSpeed{0};
  uint8_t newTimer{0};
  bool newSetting{false};

  // Fan state cache (used by publish_state - matches FanState members)
  // These are updated by handleFanSettings or control() and read by publish_state()
  // bool state; // Inherited from FanState
  // int speed; // Inherited from FanState
};

// New payload structures
// typedef struct __attribute__((packed)) { ... } RfPayloadErrorStatus;
// typedef struct __attribute__((packed)) { ... } RfPayloadFilterStatus;

}  // namespace zehnder
}  // namespace esphome

#endif /* __COMPONENT_ZEHNDER_H__ */
