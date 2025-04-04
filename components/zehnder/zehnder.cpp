#include "zehnder.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"

namespace esphome {
namespace zehnder {

#define MAX_TRANSMIT_TIME 2000

static const char *const TAG = "zehnder";

typedef struct __attribute__((packed)) {
  uint32_t networkId;
} RfPayloadNetworkJoinOpen;

typedef struct __attribute__((packed)) {
  uint32_t networkId;
} RfPayloadNetworkJoinRequest;

typedef struct __attribute__((packed)) {
  uint32_t networkId;
} RfPayloadNetworkJoinAck;

typedef struct __attribute__((packed)) {
  uint8_t speed;
  uint8_t voltage;
  uint8_t timer;
} RfPayloadFanSettings;

typedef struct __attribute__((packed)) {
  uint8_t speed;
} RfPayloadFanSetSpeed;

typedef struct __attribute__((packed)) {
  uint8_t speed;
  uint8_t timer;
} RfPayloadFanSetTimer;

typedef struct __attribute__((packed)) {
  uint8_t rx_type;          // 0x00 RX Type
  uint8_t rx_id;            // 0x01 RX ID
  uint8_t tx_type;          // 0x02 TX Type
  uint8_t tx_id;            // 0x03 TX ID
  uint8_t ttl;              // 0x04 Time-To-Live
  uint8_t command;          // 0x05 Frame type
  uint8_t parameter_count;  // 0x06 Number of parameters

  union {
    uint8_t parameters[9];                           // 0x07 - 0x0F Depends on command
    RfPayloadFanSetSpeed setSpeed;                   // Command 0x02
    RfPayloadFanSetTimer setTimer;                   // Command 0x03
    RfPayloadNetworkJoinRequest networkJoinRequest;  // Command 0x04
    RfPayloadNetworkJoinOpen networkJoinOpen;        // Command 0x06
    RfPayloadFanSettings fanSettings;                // Command 0x07
    RfPayloadNetworkJoinAck networkJoinAck;          // Command 0x0C
  } payload;
} RfFrame;

ZehnderRF::ZehnderRF(void) {}

fan::FanTraits ZehnderRF::get_traits() { return fan::FanTraits(false, true, false, this->speed_count_); }

void ZehnderRF::control(const fan::FanCall &call) {
  if (call.get_state().has_value()) {
    this->state = *call.get_state();
    ESP_LOGD(TAG, "Control has state: %u", this->state);
  }
  if (call.get_speed().has_value()) {
    this->speed = *call.get_speed();
    ESP_LOGD(TAG, "Control has speed: %u", this->speed);
  }

  this->newSetting = true;
  this->newSpeed = this->state ? this->speed : 0x00;
  this->newTimer = 0;

  this->publish_state();
}

void ZehnderRF::setup() {
  ESP_LOGCONFIG(TAG, "ZEHNDER '%s':", this->get_name().c_str());

  // Clear config
  memset(&this->config_, 0, sizeof(Config));

  uint32_t hash = fnv1_hash("zehnderrf");
  this->pref_ = global_preferences->make_preference<Config>(hash, true);
  bool loaded = this->pref_.load(&this->config_);
  if (loaded) {
    ESP_LOGD(TAG, "Config load OK. NetworkId: 0x%08X, MyDeviceId: 0x%02X, MainUnitId: 0x%02X",
             this->config_.fan_networkId, this->config_.fan_my_device_id, this->config_.fan_main_unit_id);
  } else {
    ESP_LOGW(TAG, "Config load FAILED.");
    // Ensure config is zeroed if load failed, otherwise startup check might pass incorrectly
    memset(&this->config_, 0, sizeof(Config)); 
  }

  // Set nRF905 config
  nrf905::Config rfConfig;
  rfConfig = this->rf_->getConfig();

  rfConfig.band = true;
  rfConfig.channel = 118;

  // // CRC 16
  rfConfig.crc_enable = true;
  rfConfig.crc_bits = 16;

  // // TX power 10
  rfConfig.tx_power = 10;

  // // RX power normal
  rfConfig.rx_power = nrf905::PowerNormal;

  rfConfig.rx_address = 0x89816EA9;  // ZEHNDER_NETWORK_LINK_ID;
  rfConfig.rx_address_width = 4;
  rfConfig.rx_payload_width = 16;

  rfConfig.tx_address_width = 4;
  rfConfig.tx_payload_width = 16;

  rfConfig.xtal_frequency = 16000000;  // defaults for now
  rfConfig.clkOutFrequency = nrf905::ClkOut500000;
  rfConfig.clkOutEnable = false;

  // Write config back
  this->rf_->updateConfig(&rfConfig);
  this->rf_->writeTxAddress(0x89816EA9);

  this->speed_count_ = 4;

  this->lastFanQuery_ = 0;
  this->newSetting = false;

  this->rf_->setOnTxReady([this](void) {
    ESP_LOGD(TAG, "Tx Ready");
    if (this->rfState_ == RfStateTxBusy) {
      if (this->retries_ >= 0) {
        this->msgSendTime_ = millis();
        this->rfState_ = RfStateRxWait;
      } else {
        this->rfState_ = RfStateIdle;
      }
    }
  });

  this->rf_->setOnRxComplete([this](const uint8_t *const pData, const uint8_t dataLength) {
    ESP_LOGV(TAG, "Received frame");
    this->rfHandleReceived(pData, dataLength);
  });
}

void ZehnderRF::dump_config(void) {
  ESP_LOGCONFIG(TAG, "Zehnder Fan config:");
  ESP_LOGCONFIG(TAG, "  Polling interval   %u", this->interval_);
  ESP_LOGCONFIG(TAG, "  Fan networkId      0x%08X", this->config_.fan_networkId);
  ESP_LOGCONFIG(TAG, "  Fan my device type 0x%02X", this->config_.fan_my_device_type);
  ESP_LOGCONFIG(TAG, "  Fan my device id   0x%02X", this->config_.fan_my_device_id);
  ESP_LOGCONFIG(TAG, "  Fan main_unit type 0x%02X", this->config_.fan_main_unit_type);
  ESP_LOGCONFIG(TAG, "  Fan main unit id   0x%02X", this->config_.fan_main_unit_id);

  // Log diagnostic sensors
  if (this->filter_remaining_sensor_ != nullptr) {
    ESP_LOGCONFIG(TAG, "  Filter remaining sensor: YES");
  }
  if (this->filter_runtime_sensor_ != nullptr) {
    ESP_LOGCONFIG(TAG, "  Filter runtime sensor: YES");
  }
  if (this->error_count_sensor_ != nullptr) {
    ESP_LOGCONFIG(TAG, "  Error count sensor: YES");
  }
  if (this->error_code_sensor_ != nullptr) {
    ESP_LOGCONFIG(TAG, "  Error code sensor: YES");
  }
}

void ZehnderRF::loop(void) {
  uint8_t deviceId;
  nrf905::Config rfConfig;

  // Run RF handler
  this->rfHandler();

  switch (this->state_) {
    case StateStartup:
      // Wait until started up
      if (millis() > 15000) {
        // Discovery?
        if ((this->config_.fan_networkId == 0x00000000) || (this->config_.fan_my_device_type == 0) ||
            (this->config_.fan_my_device_id == 0) || (this->config_.fan_main_unit_type == 0) ||
            (this->config_.fan_main_unit_id == 0)) {
          ESP_LOGD(TAG, "Invalid config, start paring");
          this->state_ = StateStartDiscovery;
        } else {
          ESP_LOGD(TAG, "Config data valid, start polling");

          rfConfig = this->rf_->getConfig();
          rfConfig.rx_address = this->config_.fan_networkId;
          this->rf_->updateConfig(&rfConfig);
          this->rf_->writeTxAddress(this->config_.fan_networkId);

          this->state_ = StateIdle;
          this->lastFanQuery_ = millis() - this->interval_;
        }
      }
      break;

    case StateStartDiscovery:
      deviceId = this->createDeviceID();
      this->discoveryStart(deviceId);
      break;

    case StateWaitSetSpeedConfirm:
      if (this->rfState_ == RfStateIdle) {
        ESP_LOGD(TAG, "SetSpeed TX complete, returning to Idle.");
        this->state_ = StateIdle;
        this->lastFanQuery_ = millis();
      }
      break;

    case StateWaitFanSettings:
      break;

    case StateIdle:
      if (newSetting == true) {
        ESP_LOGD(TAG, "New setting received, sending speed %u, timer %u", newSpeed, newTimer);
        this->setSpeed(newSpeed, newTimer);
        this->newSetting = false;
      } else if ((millis() - this->lastFanQuery_) > this->interval_) {
        this->queryDevice();
        this->lastFanQuery_ = millis();
      }
      break;

    default:
      ESP_LOGW(TAG, "In unknown state (0x%02X), resetting to Startup", this->state_);
      this->state_ = StateStartup;
      break;
  }

  this->publish_state();
}

void ZehnderRF::rfHandleReceived(const uint8_t *const pData, const uint8_t dataLength) {
  RfFrame *frame = (RfFrame *) pData;

  ESP_LOGD(TAG, "Current state: 0x%02X", this->state_);

  switch (this->state_) {
    case StateDiscoveryWaitForLinkRequest:
      ESP_LOGD(TAG, "DiscoverStateWaitForLinkRequest");
      if (frame->command == FAN_NETWORK_JOIN_OPEN) {
        this->handleDiscoveryLinkRequest(frame);
      } else {
        ESP_LOGW(TAG, "Discovery (WaitLink): Received unexpected frame type 0x%02X from ID 0x%02X. Data: %s", frame->command, frame->tx_id, hexArrayToStr(pData, dataLength));
      }
      break;

    case StateDiscoveryWaitForJoinResponse:
      ESP_LOGD(TAG, "DiscoverStateWaitForJoinResponse");
      if (frame->command == FAN_FRAME_0B) {
        this->handleDiscoveryJoinResponse(frame);
      } else {
        ESP_LOGW(TAG, "Discovery (WaitJoin): Received unexpected frame type 0x%02X from ID 0x%02X. Data: %s", frame->command, frame->tx_id, hexArrayToStr(pData, dataLength));
      }
      break;

    case StateDiscoveryJoinComplete:
      ESP_LOGD(TAG, "StateDiscoveryJoinComplete");
      if (frame->command == FAN_TYPE_QUERY_NETWORK) {
        this->handleDiscoveryJoinComplete(frame);
      } else {
        ESP_LOGW(TAG, "Discovery (JoinComplete): Received unknown frame type 0x%02X from ID 0x%02X on network 0x%08X. Data: %s",
                   frame->command, frame->tx_id, this->config_.fan_networkId, hexArrayToStr(pData, dataLength));
      }
      break;

    case StateWaitFanSettings:
      ESP_LOGD(TAG, "StateWaitFanSettings");
      if (frame->command == FAN_TYPE_FAN_SETTINGS) {
        this->handleFanSettings(frame);
        this->state_ = StateIdle;
        this->rfState_ = RfStateIdle;
      } else {
        ESP_LOGD(TAG, "WaitFanSettings: Received other frame type 0x%02X from ID 0x%02X. Data: %s", frame->command, frame->tx_id, hexArrayToStr(pData, dataLength));
      }
      break;

    case StateIdle:
      ESP_LOGD(TAG, "StateIdle: Received frame");
      ESP_LOGD(TAG, "Idle: Received frame type 0x%02X from ID 0x%02X type 0x%02X. Data: %s",
                 frame->command, frame->tx_id, frame->tx_type, hexArrayToStr(pData, dataLength));
      break;

    default:
      ESP_LOGW(TAG, "Received frame in unknown state (0x%02X); type 0x%02X from ID 0x%02X type 0x%02X. Data: %s",
               this->state_, frame->command, frame->tx_id, frame->tx_type, hexArrayToStr(pData, dataLength));
      break;
  }
}

void ZehnderRF::handleFanSettings(const RfFrame *const frame) {
  ESP_LOGD(TAG, "Received fan settings; speed: 0x%02X voltage: %u timer: %u", frame->payload.fanSettings.speed,
           frame->payload.fanSettings.voltage, frame->payload.fanSettings.timer);

  this->speed = frame->payload.fanSettings.speed;
  this->state = (this->speed > 0);

  this->publish_state();

  if (this->ventilation_percentage_sensor_ != nullptr) {
      this->ventilation_percentage_sensor_->publish_state(frame->payload.fanSettings.voltage);
  }
  if (this->timer_binary_sensor_ != nullptr) {
      this->timer_binary_sensor_->publish_state(frame->payload.fanSettings.timer != 0);
  }
  if (this->ventilation_mode_text_sensor_ != nullptr) {
      this->ventilation_mode_text_sensor_->publish_state(this->speedToMode_(this->speed));
  }
}

uint8_t ZehnderRF::createDeviceID(void) {
  uint8_t mac[6];
  wifi::global_wifi_component->get_mac_address_raw(mac);
  return mac[5];
}

void ZehnderRF::setSpeed(const uint8_t paramSpeed, const uint8_t paramTimer) {
  RfFrame frame;
  Result result;
  const uint8_t speed = minmax(paramSpeed, ZEHNDER_SPEED_OFF, ZEHNDER_SPEED_MAX);

  ESP_LOGD(TAG, "Set fan speed: %u", speed);

  frame.rx_type = this->config_.fan_main_unit_type;
  frame.rx_id = this->config_.fan_main_unit_id;
  frame.tx_type = this->config_.fan_my_device_type;
  frame.tx_id = this->config_.fan_my_device_id;
  frame.ttl = 0xFA;

  if (paramTimer == 0) {
    frame.command = ZEHNDER_CMD_SETSPEED;
    frame.parameter_count = sizeof(RfPayloadFanSetSpeed);
    frame.payload.setSpeed.speed = speed;
  } else {
    frame.command = ZEHNDER_CMD_SETTIMER;
    frame.parameter_count = sizeof(RfPayloadFanSetTimer);
    frame.payload.setTimer.speed = speed;
    frame.payload.setTimer.timer = paramTimer;
  }

  result = this->startTransmit((uint8_t *) &frame, 10);
  if (result == Success) {
    this->state_ = StateWaitSetSpeedConfirm;
  }
}

void ZehnderRF::queryDevice(void) {
  RfFrame frame;
  Result result;

  ESP_LOGD(TAG, "Query device");

  frame.rx_type = this->config_.fan_main_unit_type;
  frame.rx_id = this->config_.fan_main_unit_id;
  frame.tx_type = this->config_.fan_my_device_type;
  frame.tx_id = this->config_.fan_my_device_id;
  frame.ttl = 0xFA;
  frame.command = ZEHNDER_CMD_QUERY;
  frame.parameter_count = 0;

  result = this->startTransmit((uint8_t *) &frame, 10);
  if (result == Success) {
    this->state_ = StateWaitFanSettings;
  }
}

void ZehnderRF::discoveryStart(const uint8_t deviceId) {
  nrf905::Config rfConfig;

  ESP_LOGD(TAG, "Start discovery with ID %u", deviceId);

  rfConfig = this->rf_->getConfig();
  rfConfig.rx_address = ZEHNDER_NETWORK_LINK_ID;
  rfConfig.tx_retransmit = false;
  this->rf_->updateConfig(&rfConfig);
  this->rf_->writeTxAddress(ZEHNDER_NETWORK_LINK_ID);

  RfFrame frame;
  frame.rx_type = 0x00;
  frame.rx_id = 0x00;
  frame.tx_type = ZEHNDER_UNIT_TYPE_REMOTE;
  frame.tx_id = deviceId;
  frame.ttl = 0xFA;
  frame.command = ZEHNDER_CMD_JOIN_REQUEST;
  frame.parameter_count = sizeof(RfPayloadNetworkJoinRequest);
  frame.payload.networkJoinRequest.networkId = 0x00000000;

  Result result = this->startTransmit((uint8_t *) &frame, 10);
  if (result == Success) {
    this->state_ = StateDiscoveryWaitForLinkRequest;
  }
}

Result ZehnderRF::startTransmit(const uint8_t *const pData, const int8_t rxRetries,
                                const std::function<void(void)> callback) {
  Result result = ResultOk;
  unsigned long startTime;
  bool busy = true;

  if (this->rfState_ != RfStateIdle) {
    ESP_LOGW(TAG, "TX still ongoing");
    result = ResultBusy;
  } else {
    this->onReceiveTimeout_ = callback;
    this->retries_ = rxRetries;

    this->rf_->writeTxPayload(pData, FAN_FRAMESIZE);

    this->rfState_ = RfStateWaitAirwayFree;
    this->airwayFreeWaitTime_ = millis();
  }

  return result;
}

void ZehnderRF::rfComplete(void) {
  this->retries_ = -1;
  this->rfState_ = RfStateIdle;
}

void ZehnderRF::rfHandler(void) {
  switch (this->rfState_) {
    case RfStateIdle:
      break;

    case RfStateWaitAirwayFree:
      if ((millis() - this->airwayFreeWaitTime_) > 5000) {
        ESP_LOGW(TAG, "Airway too busy, giving up");
        this->rfState_ = RfStateIdle;

        if (this->onReceiveTimeout_ != NULL) {
          this->onReceiveTimeout_();
        }
      } else if (this->rf_->airwayBusy() == false) {
        ESP_LOGD(TAG, "Start TX");
        this->rf_->startTx(FAN_TX_FRAMES, nrf905::Receive);

        this->rfState_ = RfStateTxBusy;
      }
      break;

    case RfStateTxBusy:
      break;

    case RfStateRxWait:
      if ((this->retries_ >= 0) && ((millis() - this->msgSendTime_) > FAN_REPLY_TIMEOUT)) {
        ESP_LOGD(TAG, "Receive timeout");

        if (this->retries_ > 0) {
          --this->retries_;
          ESP_LOGD(TAG, "No data received, retry again (left: %u)", this->retries_);

          delay(150);

          this->rfState_ = RfStateWaitAirwayFree;
          this->airwayFreeWaitTime_ = millis();
        } else if (this->retries_ == 0) {
          ESP_LOGD(TAG, "No messages received, giving up now...");
          if (this->onReceiveTimeout_ != NULL) {
            this->onReceiveTimeout_();
          }

          delay(250);

          this->rfState_ = RfStateIdle;
        }
      }
      break;

    default:
      break;
  }
}

}  // namespace zehnder
}  // namespace esphome
