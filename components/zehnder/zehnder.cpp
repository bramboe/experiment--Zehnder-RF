#include "zehnder.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"
#include "esphome/components/wifi/wifi_component.h"
#include "esphome/components/network/util.h"
#include <string>
#include <algorithm>
#include <vector>

namespace esphome {
namespace zehnder {

static const char *const TAG = "zehnder";

// Helper function: Convert byte array to hex string
static std::string bytes_to_hex(const uint8_t *data, size_t len) {
  std::string res = "";
  const char *hex_chars = "0123456789ABCDEF";
  for (size_t i = 0; i < len; ++i) {
    if (i > 0)
      res += " ";
    res += hex_chars[data[i] >> 4];
    res += hex_chars[data[i] & 0x0F];
  }
  return res;
}

// Helper function: Clamp value between min and max
static uint8_t clamp(const uint8_t value, const uint8_t min_val, const uint8_t max_val) {
  return std::min(std::max(value, min_val), max_val);
}

// Constructor
ZehnderRF::ZehnderRF(void) {}

// Define Fan Traits
fan::FanTraits ZehnderRF::get_traits() { return fan::FanTraits(false, true, false, this->speed_count_); }

// Handle Fan Control Calls from Home Assistant
void ZehnderRF::control(const fan::FanCall &call) {
  if (call.get_state().has_value()) {
    this->state = *call.get_state(); // Update internal ON/OFF state
    ESP_LOGD(TAG, "Control call: State=%s", ONOFF(this->state));
  }
  if (call.get_speed().has_value()) {
    this->speed = *call.get_speed(); // Update internal speed level
    ESP_LOGD(TAG, "Control call: Speed=%d", this->speed);
  }

  // Mark that a new setting needs to be sent
  this->newSetting = true;
  this->newSpeed = this->state ? this->speed : FAN_SPEED_AUTO; // Map ON/OFF state and speed level
  this->newTimer = 0; // Timer control not implemented via standard fan call yet

  this->publish_state(); // Optimistically publish the state
}

// Component Setup
void ZehnderRF::setup() {
  ESP_LOGCONFIG(TAG, "Setting up ZehnderRF '%s'...", this->get_name().c_str());

  // Load configuration from preferences
  memset(&this->config_, 0, sizeof(Config));
  uint32_t hash = fnv1_hash("zehnderrf_config"); // Use a unique key
  this->pref_ = global_preferences->make_preference<Config>(hash, true);
  if (this->pref_.load(&this->config_)) {
    ESP_LOGD(TAG, "Loaded config - NetworkId: 0x%08X, MyDeviceId: 0x%02X, MainUnitId: 0x%02X",
             this->config_.fan_networkId, this->config_.fan_my_device_id, this->config_.fan_main_unit_id);
  } else {
    ESP_LOGW(TAG, "Failed to load config. Starting fresh.");
    memset(&this->config_, 0, sizeof(Config));
  }

  // Configure nRF905 Radio
  nrf905::Config rfConfig = this->rf_->getConfig(); // Get current config (defaults)
  rfConfig.band = true; // 868 MHz band
  rfConfig.channel = 118;
  rfConfig.crc_enable = true;
  rfConfig.crc_bits = 16;
  rfConfig.tx_power = 10; // Max power
  rfConfig.rx_power = nrf905::PowerNormal;
  rfConfig.rx_address_width = 4;
  rfConfig.tx_address_width = 4;
  rfConfig.rx_payload_width = FAN_FRAMESIZE;
  rfConfig.tx_payload_width = FAN_FRAMESIZE;
  rfConfig.xtal_frequency = 16000000;
  rfConfig.clkOutFrequency = nrf905::ClkOut500000;
  rfConfig.clkOutEnable = false;
  rfConfig.auto_retransmit = false; // Start with retransmit off (used in discovery)
  rfConfig.rx_address = NETWORK_LINK_ID; // Initial address for discovery

  this->rf_->updateConfig(&rfConfig); // Apply configuration
  this->rf_->writeTxAddress(NETWORK_LINK_ID); // Set initial TX address

  this->speed_count_ = 4; // Number of speed presets (Low, Medium, High, Max)
  this->state_ = StateStartup; // Set initial state machine state
  this->lastFanQuery_ = 0;
  this->newSetting = false;
  this->rfState_ = RfStateIdle;

  // Setup nRF905 callbacks
  this->rf_->setOnTxReady([this](void) {
    ESP_LOGV(TAG, "nRF905: TX Ready");
    if (this->rfState_ == RfStateTxBusy) {
      if (this->retries_ >= 0) { // If we expect a reply
        this->msgSendTime_ = millis();
        this->rfState_ = RfStateRxWait; // Move to wait for reply state
      } else { // If no reply expected
        this->rfState_ = RfStateIdle; // TX done, return to idle
        // If we were waiting for TX confirmation, now we can go idle
        if(this->state_ == StateWaitSetSpeedConfirm) {
            ESP_LOGD(TAG, "SetSpeed TX complete, returning to Idle state.");
            this->state_ = StateIdle;
        }
      }
    }
  });

  this->rf_->setOnRxComplete([this](const uint8_t *const pData, const uint8_t dataLength) {
    ESP_LOGV(TAG, "nRF905: RX Complete");
    this->rfHandleReceived(pData, dataLength);
  });

  ESP_LOGCONFIG(TAG, "ZehnderRF setup complete.");
}

// Dump Configuration
void ZehnderRF::dump_config(void) {
  ESP_LOGCONFIG(TAG, "ZehnderRF Component Configuration:");
  ESP_LOGCONFIG(TAG, "  Configured Update Interval: %u ms", this->interval_);
  ESP_LOGCONFIG(TAG, "  Paired Network ID: 0x%08X", this->config_.fan_networkId);
  ESP_LOGCONFIG(TAG, "  My Device Type: 0x%02X", this->config_.fan_my_device_type);
  ESP_LOGCONFIG(TAG, "  My Device ID: 0x%02X", this->config_.fan_my_device_id);
  ESP_LOGCONFIG(TAG, "  Main Unit Type: 0x%02X", this->config_.fan_main_unit_type);
  ESP_LOGCONFIG(TAG, "  Main Unit ID: 0x%02X", this->config_.fan_main_unit_id);
  LOG_SENSOR("  ", "Ventilation Percentage Sensor", this->ventilation_percentage_sensor_);
  LOG_BINARY_SENSOR("  ", "Timer Binary Sensor", this->timer_binary_sensor_);
  LOG_TEXT_SENSOR("  ", "Ventilation Mode Sensor", this->ventilation_mode_text_sensor_);
  LOG_SENSOR("  ", "Filter Remaining Sensor", this->filter_remaining_sensor_);
  LOG_SENSOR("  ", "Filter Runtime Sensor", this->filter_runtime_sensor_);
  LOG_SENSOR("  ", "Error Count Sensor", this->error_count_sensor_);
  LOG_TEXT_SENSOR("  ", "Error Code Sensor", this->error_code_sensor_);
}

// Main Loop Logic
void ZehnderRF::loop(void) {
  this->rfHandler(); // Process RF state machine (timeouts, etc.)

  uint8_t deviceId;
  nrf905::Config rfConfig;

  // Main state machine
  switch (this->state_) {
    case StateStartup:
      // Give some time for system to stabilize after boot
      if (millis() > 10000) { // Wait 10 seconds
        // Check if we have valid pairing info
        if ((this->config_.fan_networkId == 0x00000000) || (this->config_.fan_my_device_type == 0) ||
            (this->config_.fan_my_device_id == 0) || (this->config_.fan_main_unit_type == 0) ||
            (this->config_.fan_main_unit_id == 0)) {
          ESP_LOGI(TAG, "No valid pairing config found. Starting discovery...");
          this->state_ = StateStartDiscovery;
        } else {
          ESP_LOGI(TAG, "Valid pairing config found. Starting normal operation.");
          // Apply paired configuration to radio
          rfConfig = this->rf_->getConfig();
          rfConfig.rx_address = this->config_.fan_networkId;
          rfConfig.auto_retransmit = true; // Enable retransmit for normal operation
          this->rf_->updateConfig(&rfConfig);
          this->rf_->writeTxAddress(this->config_.fan_networkId);
          this->state_ = StateIdle;
          this->lastFanQuery_ = millis() - this->interval_; // Force initial query soon
        }
      }
      break;

    case StateStartDiscovery:
      deviceId = this->createDeviceID();
      this->discoveryStart(deviceId); // Will attempt transmit and change state
      break;

    // Discovery states are progressed via received packets in rfHandleReceived
    case StateDiscoveryWaitForLinkRequest:
    case StateDiscoveryWaitForJoinResponse:
    case StateDiscoveryJoinComplete:
      // Waiting for incoming messages or timeout via rfHandler
      break;

    case StateWaitSetSpeedConfirm:
      // Waiting for OnTxReady callback to confirm TX finished.
      // State change happens in the OnTxReady callback.
      break;

    case StateWaitFanSettings:
      // Waiting for FAN_TYPE_FAN_SETTINGS reply.
      // State change happens in rfHandleReceived or timeout via rfHandler.
      break;

    case StateIdle:
      // Check if a command from Home Assistant is pending
      if (this->newSetting) {
        ESP_LOGD(TAG, "Idle: New setting pending (Speed: %d), sending command.", this->newSpeed);
        this->setSpeed(this->newSpeed, this->newTimer);
        this->newSetting = false; // Clear the flag
      } 
      // Otherwise, check if it's time for a periodic status query
      else if ((millis() - this->lastFanQuery_) >= this->interval_) {
        ESP_LOGD(TAG, "Idle: Polling interval reached. Querying device status.");
        this->queryDevice();
        this->lastFanQuery_ = millis(); // Reset timer *after* initiating query
      }
      break;

    default:
      ESP_LOGW(TAG, "Reached unknown state (0x%02X)! Resetting to Startup.", this->state_);
      this->state_ = StateStartup;
      break;
  }
}

// Handle Received RF Data
void ZehnderRF::rfHandleReceived(const uint8_t *const pData, const uint8_t dataLength) {
  if (dataLength < sizeof(RfFrame)) { // Basic sanity check
      ESP_LOGW(TAG, "Received short frame (%d bytes), discarding.", dataLength);
      return;
  }
  const RfFrame *frame = (const RfFrame *) pData;

  ESP_LOGD(TAG, "Received Frame in State %d. Cmd: 0x%02X, From: %02X:%02X, To: %02X:%02X, Data: %s", 
           this->state_, frame->command, frame->tx_type, frame->tx_id, frame->rx_type, frame->rx_id, 
           bytes_to_hex(pData, dataLength).c_str());

  // State-specific handling
  switch (this->state_) {
    case StateDiscoveryWaitForLinkRequest: // Expecting JOIN_OPEN (0x06)
      ESP_LOGV(TAG, "Handling received frame in StateDiscoveryWaitForLinkRequest");
      if (frame->command == FAN_NETWORK_JOIN_OPEN) {
        this->handleDiscoveryLinkRequest(frame);
      } else {
        ESP_LOGW(TAG, "Discovery (WaitLink): Received unexpected cmd 0x%02X", frame->command);
      }
      break;

    case StateDiscoveryWaitForJoinResponse: // Expecting 0x0B (ACK?)
      ESP_LOGV(TAG, "Handling received frame in StateDiscoveryWaitForJoinResponse");
      if (frame->command == FAN_FRAME_0B) {
        this->handleDiscoveryJoinResponse(frame);
      } else {
        ESP_LOGW(TAG, "Discovery (WaitJoin): Received unexpected cmd 0x%02X", frame->command);
      }
      break;

    case StateDiscoveryJoinComplete: // Expecting QUERY_NETWORK (0x0D)
      ESP_LOGV(TAG, "Handling received frame in StateDiscoveryJoinComplete");
      if (frame->command == FAN_TYPE_QUERY_NETWORK) {
        this->handleDiscoveryJoinComplete(frame);
      } else {
        ESP_LOGW(TAG, "Discovery (JoinComplete): Received unexpected cmd 0x%02X", frame->command);
      }
      break;

    case StateWaitFanSettings: // Expecting FAN_SETTINGS (0x07)
      ESP_LOGV(TAG, "Handling received frame in StateWaitFanSettings");
      if (frame->command == FAN_TYPE_FAN_SETTINGS) {
        this->handleFanSettings(frame);
        this->state_ = StateIdle; // Got response, return to idle
        this->rfState_ = RfStateIdle;
        this->rfComplete(); // Mark RF layer as idle too
      } else {
        ESP_LOGD(TAG, "WaitFanSettings: Received other cmd 0x%02X while waiting for 0x07", frame->command);
      }
      break;

    case StateIdle:
      ESP_LOGV(TAG, "Handling received frame in StateIdle");
      ESP_LOGD(TAG, "Idle: Received frame. Cmd: 0x%02X, From: %02X:%02X", frame->command, frame->tx_type, frame->tx_id);
      if (frame->command == FAN_TYPE_FAN_SETTINGS) {
           if (frame->tx_type == this->config_.fan_main_unit_type && frame->tx_id == this->config_.fan_main_unit_id) {
                ESP_LOGD(TAG,"Idle: Received unsolicited fan settings update.");
                this->handleFanSettings(frame);
           }
      }
      break;

    case StateStartup:
    case StateStartDiscovery:
    case StateWaitSetSpeedConfirm:
    default:
      ESP_LOGD(TAG, "Received frame ignored in current state (%d).", this->state_);
      break;
  }
}

// Definition of Discovery/Settings Handler Functions
void ZehnderRF::handleDiscoveryLinkRequest(const RfFrame *const frame) {
  if (frame->tx_type != FAN_TYPE_MAIN_UNIT) {
      ESP_LOGW(TAG, "Discovery: JOIN_OPEN received from non-main unit type 0x%02X. Ignoring.", frame->tx_type);
      return;
  }
  ESP_LOGI(TAG, "Discovery Step 1: Found Main Unit (Type: 0x%02X, ID: 0x%02X) on Network 0x%08X. Sending Join Request...",
           frame->tx_type, frame->tx_id, frame->payload.networkJoinOpen.networkId);

  this->rfComplete(); // Mark RF as idle before starting new TX
  
  this->config_.fan_networkId = frame->payload.networkJoinOpen.networkId;
  this->config_.fan_main_unit_type = frame->tx_type;
  this->config_.fan_main_unit_id = frame->tx_id;
  this->config_.fan_my_device_type = FAN_TYPE_REMOTE_CONTROL;

  nrf905::Config rfConfig = this->rf_->getConfig();
  rfConfig.rx_address = this->config_.fan_networkId;
  rfConfig.auto_retransmit = true;
  this->rf_->updateConfig(&rfConfig);
  this->rf_->writeTxAddress(this->config_.fan_networkId);

  RfFrame txFrame;
  memset(&txFrame, 0, sizeof(RfFrame));
  txFrame.rx_type = this->config_.fan_main_unit_type;
  txFrame.rx_id = this->config_.fan_main_unit_id;
  txFrame.tx_type = this->config_.fan_my_device_type;
  txFrame.tx_id = this->config_.fan_my_device_id;
  txFrame.ttl = FAN_TTL;
  txFrame.command = FAN_NETWORK_JOIN_REQUEST;
  txFrame.parameter_count = sizeof(RfPayloadNetworkJoinRequest);
  txFrame.payload.networkJoinRequest.networkId = this->config_.fan_networkId;

  Result result = this->startTransmit((uint8_t *)&txFrame, FAN_TX_RETRIES, [this]() {
    ESP_LOGW(TAG, "Timeout waiting for Join ACK (0x0B). Retrying discovery.");
    this->state_ = StateStartDiscovery;
  });

  if (result == ResultOk) {
    this->state_ = StateDiscoveryWaitForJoinResponse;
  } else {
    ESP_LOGE(TAG, "Failed to transmit Join Request (0x04). Retrying discovery.");
    App.scheduler.set_timeout(this, "discovery_retry_1", 2000, [this](){ this->state_ = StateStartDiscovery; });
  }
}

void ZehnderRF::handleDiscoveryJoinResponse(const RfFrame *const frame) {
    RfFrame txFrame;
    if ((frame->rx_type == FAN_TYPE_REMOTE_CONTROL) && (frame->rx_id == this->config_.fan_my_device_id) &&
        (frame->tx_type == FAN_TYPE_MAIN_UNIT) && (frame->tx_id == this->config_.fan_main_unit_id)) {
        ESP_LOGI(TAG, "Discovery Step 2: Received Join ACK (0x0B) from Main Unit. Sending Final ACK (0x0B)...", frame->tx_id);
        this->rfComplete();
        memset(&txFrame, 0, sizeof(RfFrame));
        txFrame.rx_type = FAN_TYPE_MAIN_UNIT;
        txFrame.rx_id = this->config_.fan_main_unit_id;
        txFrame.tx_type = FAN_TYPE_REMOTE_CONTROL;
        txFrame.tx_id = this->config_.fan_my_device_id;
        txFrame.ttl = FAN_TTL;
        txFrame.command = FAN_FRAME_0B;
        txFrame.parameter_count = 0x00;
        Result result = this->startTransmit((uint8_t *)&txFrame, FAN_TX_RETRIES, [this]() {
            ESP_LOGW(TAG, "Timeout waiting for Join Success (0x0D). Retrying discovery.");
            this->state_ = StateStartDiscovery;
        });
        if (result == ResultOk) {
            this->state_ = StateDiscoveryJoinComplete;
        } else {
            ESP_LOGE(TAG, "Failed to transmit Final ACK (0x0B). Retrying discovery.");
            App.scheduler.set_timeout(this, "discovery_retry_2", 2000, [this](){ this->state_ = StateStartDiscovery; });
        }
    } else {
        ESP_LOGW(TAG, "Discovery (WaitJoin): Received 0x0B with mismatched ID/Type. RX_T:%02X RX_ID:%02X TX_T:%02X TX_ID:%02X",
                  frame->rx_type, frame->rx_id, frame->tx_type, frame->tx_id);
    }
}

void ZehnderRF::handleDiscoveryJoinComplete(const RfFrame *const frame) {
    if ((frame->rx_type == FAN_TYPE_REMOTE_CONTROL) && (frame->rx_id == this->config_.fan_my_device_id) &&
        (frame->tx_type == FAN_TYPE_MAIN_UNIT) && (frame->tx_id == this->config_.fan_main_unit_id)) {
        ESP_LOGI(TAG, "Discovery Step 3: Received Join Success (0x0D) from Main Unit. Pairing complete!");
        this->rfComplete();
        this->config_.fan_my_device_type = FAN_TYPE_REMOTE_CONTROL;
        ESP_LOGD(TAG, "Saving config: Net:0x%08X, Me:%02X:%02X, Main:%02X:%02X",
                 this->config_.fan_networkId, this->config_.fan_my_device_type, this->config_.fan_my_device_id,
                 this->config_.fan_main_unit_type, this->config_.fan_main_unit_id);
        if (!this->pref_.save(&this->config_)) {
            ESP_LOGE(TAG, "Failed to save pairing configuration to flash!");
        }
        this->state_ = StateIdle;
        this->lastFanQuery_ = millis() - this->interval_ + 500;
    } else {
        ESP_LOGW(TAG, "Discovery (JoinComplete): Received 0x0D with mismatched ID/Type. RX_T:%02X RX_ID:%02X TX_T:%02X TX_ID:%02X",
                  frame->rx_type, frame->rx_id, frame->tx_type, frame->tx_id);
    }
}

void ZehnderRF::handleFanSettings(const RfFrame *const frame) {
  if (frame->tx_type != this->config_.fan_main_unit_type || frame->tx_id != this->config_.fan_main_unit_id) {
      ESP_LOGW(TAG,"Received Fan Settings from unexpected source (%02X:%02X). Ignoring.", frame->tx_type, frame->tx_id);
      return;
  }
  const RfPayloadFanSettings *settings = &frame->payload.fanSettings;
  ESP_LOGD(TAG, "Received Fan Settings - Speed: 0x%02X, Voltage: %u%%, Timer: %u",
           settings->speed, settings->voltage, settings->timer);
  this->speed = settings->speed;
  this->state = (this->speed > FAN_SPEED_AUTO);
  this->voltage = settings->voltage;
  this->timer = (settings->timer != 0);
  this->publish_state();
  if (this->ventilation_percentage_sensor_ != nullptr) {
    this->ventilation_percentage_sensor_->publish_state(this->voltage);
  }
  if (this->timer_binary_sensor_ != nullptr) {
    this->timer_binary_sensor_->publish_state(this->timer);
  }
  if (this->ventilation_mode_text_sensor_ != nullptr) {
    this->ventilation_mode_text_sensor_->publish_state(this->speedToMode_(this->speed));
  }
}

// Helper: Convert speed preset to text mode
std::string ZehnderRF::speedToMode_(uint8_t speed_preset) {
    switch (speed_preset) {
        case FAN_SPEED_AUTO: return "Auto";
        case FAN_SPEED_LOW: return "Low";
        case FAN_SPEED_MEDIUM: return "Medium";
        case FAN_SPEED_HIGH: return "High";
        case FAN_SPEED_MAX: return "Max";
        default: return "Unknown";
    }
}

// Generate a unique-ish device ID based on MAC
uint8_t ZehnderRF::createDeviceID(void) {
  // Correct way to get MAC address string
  std::string mac_address = esphome::network::get_mac_address();
  ESP_LOGV(TAG, "Using MAC address for ID: %s", mac_address.c_str());
  // Extract the last byte (two hex characters)
  std::string last_byte_str = mac_address.substr(mac_address.length() - 2, 2);
  uint8_t id = (uint8_t) strtol(last_byte_str.c_str(), nullptr, 16);
  ESP_LOGV(TAG, "Generated potential ID: 0x%02X", id);
  return clamp(id, 1, 254); // Avoid 0x00 and 0xFF
}

// Send Speed/Timer Command
void ZehnderRF::setSpeed(const uint8_t paramSpeed, const uint8_t paramTimer) {
  if (this->config_.fan_networkId == 0) { // Don't send commands if not paired
      ESP_LOGW(TAG, "Cannot set speed: Not paired.");
      return;
  }
  const uint8_t speed_clamped = clamp(paramSpeed, FAN_SPEED_AUTO, FAN_SPEED_MAX);
  ESP_LOGD(TAG, "Sending Set Speed/Timer command - Speed: %u, Timer: %u", speed_clamped, paramTimer);

  RfFrame frame;
  memset(&frame, 0, sizeof(RfFrame));
  frame.rx_type = this->config_.fan_main_unit_type;
  frame.rx_id = this->config_.fan_main_unit_id;
  frame.tx_type = this->config_.fan_my_device_type;
  frame.tx_id = this->config_.fan_my_device_id;
  frame.ttl = FAN_TTL;

  if (paramTimer == 0) {
    frame.command = FAN_FRAME_SETSPEED;
    frame.parameter_count = sizeof(RfPayloadFanSetSpeed);
    frame.payload.setSpeed.speed = speed_clamped;
  } else {
    frame.command = FAN_FRAME_SETTIMER;
    frame.parameter_count = sizeof(RfPayloadFanSetTimer);
    frame.payload.setTimer.speed = speed_clamped;
    frame.payload.setTimer.timer = paramTimer;
  }

  // Send the frame, no reply expected according to some traces, just TX confirmation needed.
  // Use -1 for retries to indicate no reply needed.
  Result result = this->startTransmit((uint8_t *) &frame, -1);
  if (result == ResultOk) {
    this->state_ = StateWaitSetSpeedConfirm; // Wait for TX confirmation
  } else {
    ESP_LOGW(TAG, "Failed to start transmit for setSpeed. RF state: %d", this->rfState_);
    // If busy, the pending setting flag (`newSetting`) should still be true,
    // so loop() will try again later.
  }
}

// Send Device Status Query Command
void ZehnderRF::queryDevice(void) {
  if (this->config_.fan_networkId == 0) { // Don't send commands if not paired
      ESP_LOGW(TAG, "Cannot query device: Not paired.");
      return;
  }
  ESP_LOGD(TAG, "Sending Query Device command (0x10)...");

  RfFrame frame;
  memset(&frame, 0, sizeof(RfFrame));
  frame.rx_type = this->config_.fan_main_unit_type;
  frame.rx_id = this->config_.fan_main_unit_id;
  frame.tx_type = this->config_.fan_my_device_type;
  frame.tx_id = this->config_.fan_my_device_id;
  frame.ttl = FAN_TTL;
  frame.command = FAN_TYPE_QUERY_DEVICE;
  frame.parameter_count = 0;

  // Send the frame, expect FAN_SETTINGS (0x07) reply
  Result result = this->startTransmit((uint8_t *) &frame, FAN_TX_RETRIES, [this]() {
    ESP_LOGW(TAG, "Timeout waiting for Fan Settings (0x07) reply.");
    this->state_ = StateIdle; // Return to idle on timeout
  });

  if (result == ResultOk) {
    this->state_ = StateWaitFanSettings; // Wait for the reply
  } else {
    ESP_LOGW(TAG, "Failed to start transmit for queryDevice. RF state: %d", this->rfState_);
  }
}

// Start Discovery Process
void ZehnderRF::discoveryStart(const uint8_t deviceId) {
  ESP_LOGI(TAG, "Starting Discovery with potential ID %u...", deviceId);

  // Store potential ID and type
  this->config_.fan_my_device_id = deviceId;
  this->config_.fan_my_device_type = FAN_TYPE_REMOTE_CONTROL;

  // Configure radio for discovery broadcast
  nrf905::Config rfConfig = this->rf_->getConfig();
  rfConfig.rx_address = NETWORK_LINK_ID;
  rfConfig.auto_retransmit = false; // No retransmit for broadcast
  this->rf_->updateConfig(&rfConfig);
  this->rf_->writeTxAddress(NETWORK_LINK_ID);

  // Prepare Discovery Broadcast Frame (JOIN_REQUEST 0x04)
  RfFrame frame;
  memset(&frame, 0, sizeof(RfFrame));
  frame.rx_type = 0x00; // Broadcast
  frame.rx_id = 0x00;
  frame.tx_type = FAN_TYPE_REMOTE_CONTROL; // Our type
  frame.tx_id = deviceId;
  frame.ttl = FAN_TTL;
  frame.command = FAN_NETWORK_JOIN_REQUEST;
  frame.parameter_count = sizeof(RfPayloadNetworkJoinRequest);
  frame.payload.networkJoinRequest.networkId = 0x00000000; // Network ID not known

  // Send the broadcast, expect JOIN_OPEN (0x06) reply
  Result result = this->startTransmit((uint8_t *) &frame, FAN_TX_RETRIES, [this]() {
    ESP_LOGW(TAG, "Timeout waiting for Join Open (0x06) response. Retrying discovery...");
    App.scheduler.set_timeout(this, "discovery_retry_0", 5000, [this](){ this->state_ = StateStartDiscovery; });
  });

  if (result == ResultOk) {
    this->state_ = StateDiscoveryWaitForLinkRequest; // Move to wait for reply state
  } else {
    ESP_LOGE(TAG, "Failed to start discovery broadcast (0x04). Retrying discovery...");
    App.scheduler.set_timeout(this, "discovery_retry_0b", 5000, [this](){ this->state_ = StateStartDiscovery; });
  }
}

// Initiate RF Transmission
Result ZehnderRF::startTransmit(const uint8_t *const pData, const int8_t rxRetries,
                                const std::function<void(void)> timeoutCallback) {
  if (this->rfState_ != RfStateIdle) {
    ESP_LOGW(TAG, "Cannot start transmit: RF layer busy (State: %d)", this->rfState_);
    return ResultBusy;
  }

  ESP_LOGV(TAG, "Starting transmit. Retries=%d, Data: %s", rxRetries, bytes_to_hex(pData, FAN_FRAMESIZE).c_str());
  this->onReceiveTimeout_ = timeoutCallback;
  this->retries_ = rxRetries;

  this->rf_->writeTxPayload(pData, FAN_FRAMESIZE);

  // Move to wait for airway clear state
  this->rfState_ = RfStateWaitAirwayFree;
  this->airwayFreeWaitTime_ = millis();
  return ResultOk;
}

// Mark RF Transmission/Reception Cycle as Complete
void ZehnderRF::rfComplete(void) {
  ESP_LOGV(TAG, "Marking RF cycle complete.");
  this->retries_ = -1; // No more retries needed
  this->rfState_ = RfStateIdle;
  // Do not change the main state_ here, let the calling function do that.
}

// RF Layer State Machine (Handles Timing, Retries, Airway Check)
void ZehnderRF::rfHandler(void) {
  switch (this->rfState_) {
    case RfStateIdle:
      // Nothing to do
      break;

    case RfStateWaitAirwayFree:
      // Check if airway is clear or timeout waiting
      if ((millis() - this->airwayFreeWaitTime_) > 5000) { // 5 second timeout for airway clear
        ESP_LOGW(TAG, "Airway busy timeout! Aborting TX.");
        if (this->onReceiveTimeout_ != nullptr) {
          this->onReceiveTimeout_(); // Trigger timeout callback
        }
        this->rfState_ = RfStateIdle; // Give up
      } else if (!this->rf_->airwayBusy()) {
        ESP_LOGV(TAG, "Airway clear. Starting TX...");
        // Expect reply? Then set next mode to Receive. No reply? Set next mode to Idle.
        nrf905::Mode next_mode = (this->retries_ >= 0) ? nrf905::Receive : nrf905::Idle;
        this->rf_->startTx(FAN_TX_FRAMES, next_mode);
        this->rfState_ = RfStateTxBusy;
      }
      break;

    case RfStateTxBusy:
      // Waiting for the OnTxReady callback from nRF905 component
      break;

    case RfStateRxWait:
      // Waiting for OnRxComplete callback, or timeout
      if ((this->retries_ >= 0) && ((millis() - this->msgSendTime_) > FAN_REPLY_TIMEOUT)) {
        ESP_LOGD(TAG, "Timeout waiting for RX reply.");
        if (this->retries_ > 0) {
          this->retries_--;
          ESP_LOGD(TAG, "Retrying transmission (retries left: %d)...", this->retries_);
          delay(150); // Short delay before retrying
          this->rfState_ = RfStateWaitAirwayFree; // Go back to check airway
          this->airwayFreeWaitTime_ = millis();
        } else { // retries_ == 0
          ESP_LOGW(TAG, "No reply received after all retries. Giving up.");
          if (this->onReceiveTimeout_ != nullptr) {
            this->onReceiveTimeout_(); // Trigger the final timeout callback
          }
          this->rfState_ = RfStateIdle; // Return RF layer to Idle
          // If the main state machine was waiting for this reply, reset it too
           if (this->state_ == StateWaitFanSettings || this->state_ == StateDiscoveryWaitForLinkRequest || 
              this->state_ == StateDiscoveryWaitForJoinResponse || this->state_ == StateDiscoveryJoinComplete) {
              ESP_LOGW(TAG, "Timeout waiting for response in state %d, returning to StateIdle.", this->state_);
              this->state_ = StateIdle;
          }
        }
      }
      break;
  }
}

} // namespace zehnder
} // namespace esphome
