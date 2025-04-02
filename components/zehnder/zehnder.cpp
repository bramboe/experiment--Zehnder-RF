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

  switch (this->state_) {
    case StateIdle:
      // Set speed
      this->setSpeed(this->state ? this->speed : 0x00, 0);

      this->lastFanQuery_ = millis();  // Update time
      break;

    default:
      break;
  }

  this->publish_state();
}

void ZehnderRF::setup() {
  ESP_LOGCONFIG(TAG, "ZEHNDER '%s': (Setup TEMPORARILY GUTTED for stability test)", this->get_name().c_str());

  // Set a default state to prevent issues if loop is ever re-enabled partially
  this->state_ = StateIdle;
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
  /* --- Start of Gutted Code ---
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
          ESP_LOGW(TAG, "Invalid config / Not paired. Please trigger pairing manually (using Reset Pairing button).");
          // Stay idle until pairing is triggered manually. Do not start discovery automatically.
          this->state_ = StateIdle; 
        } else {
          ESP_LOGD(TAG, "Config data valid, setting network ID and preparing for communication.");

          rfConfig = this->rf_->getConfig();
          rfConfig.rx_address = this->config_.fan_networkId;
          this->rf_->updateConfig(&rfConfig, NULL);
          this->rf_->writeTxAddress(this->config_.fan_networkId, NULL);
          // Set mode to receive based on the valid config
          this->rf_->setMode(nrf905::Receive);

          this->state_ = StateIdle;
          // Reset query timers - Note: periodic queries are currently commented out
          this->lastFanQuery_ = 0;  
          this->lastFilterQuery_ = 0; 
          this->lastErrorQuery_ = 0;  
        }
      }
      break;

    case StateStartDiscovery:
      ESP_LOGD(TAG, "Starting discovery");

      // Connect receiver to network link id
      rfConfig = this->rf_->getConfig();
      rfConfig.rx_address = NETWORK_LINK_ID;
      this->rf_->updateConfig(&rfConfig, NULL);
      this->rf_->writeTxAddress(NETWORK_LINK_ID, NULL);
      // this->rf_->startReceive();
      this->rf_->setMode(nrf905::Receive);

      // Keep generating different IDs to prevent collision with
      // other devices also looking for a main unit
      deviceId = this->createDeviceID();

      // Start discovery
      this->discoveryStart(deviceId);
      break;

    case StateIdle:
      // Temporarily commented out for stability testing
      if ((millis() - this->lastFanQuery_) >= this->interval_) {
        // Time to send a new query
        this->queryDevice();
      }
      
      // Query filter status every hour or according to config
      if ((millis() - this->lastFilterQuery_) >= 3600000) {  // 1 hour = 3600000 ms
        this->queryFilterStatus();
      }

      // Query error status every 5 minutes
      if ((millis() - this->lastErrorQuery_) >= 300000) {  // 5 minutes = 300000 ms
        this->queryErrorStatus();
      }
      
      
      // Check and apply any new speed settings
      if (this->newSetting) {
        this->setSpeed(this->newSpeed, this->newTimer);
        this->newSetting = false;
      }
      break;

    default:
      break;
  }
  --- End of Gutted Code --- */
}

void ZehnderRF::queryErrorStatus(void) {
  RfFrame *const pFrame = (RfFrame *) this->_txFrame;  // frame helper

  ESP_LOGD(TAG, "Query error status");

  (void) memset(this->_txFrame, 0, FAN_FRAMESIZE);  // Clear frame data

  pFrame->rx_type = this->config_.fan_main_unit_type;  // Main
  pFrame->rx_id = this->config_.fan_main_unit_id;
  pFrame->tx_type = this->config_.fan_my_device_type;
  pFrame->tx_id = this->config_.fan_my_device_id;
  pFrame->ttl = FAN_TTL;
  pFrame->command = FAN_TYPE_QUERY_ERROR_STATUS;  // Request error status
  pFrame->parameter_count = 0x00;                // No params

  this->startTransmit(this->_txFrame, FAN_TX_RETRIES, [this]() {
    ESP_LOGW(TAG, "Error status query timeout");
    this->state_ = StateIdle;
  });

  this->lastErrorQuery_ = millis();  // Update time
  this->state_ = StateWaitErrorStatusResponse;
}

void ZehnderRF::queryFilterStatus(void) {
  RfFrame *const pFrame = (RfFrame *) this->_txFrame;  // frame helper

  ESP_LOGD(TAG, "Query filter status");

  (void) memset(this->_txFrame, 0, FAN_FRAMESIZE);  // Clear frame data

  pFrame->rx_type = this->config_.fan_main_unit_type;  // Main
  pFrame->rx_id = this->config_.fan_main_unit_id;
  pFrame->tx_type = this->config_.fan_my_device_type;
  pFrame->tx_id = this->config_.fan_my_device_id;
  pFrame->ttl = FAN_TTL;
  pFrame->command = FAN_TYPE_QUERY_FILTER_STATUS;  // Request filter status
  pFrame->parameter_count = 0x00;                  // No params

  this->startTransmit(this->_txFrame, FAN_TX_RETRIES, [this]() {
    ESP_LOGW(TAG, "Filter status query timeout");
    this->state_ = StateIdle;
  });

  this->lastFilterQuery_ = millis();  // Update time
  this->state_ = StateWaitFilterStatusResponse;
}

void ZehnderRF::rfHandleReceived(const uint8_t *const pData, const uint8_t dataLength) {
  const RfFrame *const pResponse = (RfFrame *) pData;
  RfFrame *const pTxFrame = (RfFrame *) this->_txFrame;  // frame helper
  nrf905::Config rfConfig;

  ESP_LOGD(TAG, "Current state: 0x%02X", this->state_);
  switch (this->state_) {
    case StateDiscoveryWaitForLinkRequest:
      ESP_LOGD(TAG, "DiscoverStateWaitForLinkRequest");
      switch (pResponse->command) {
        case FAN_NETWORK_JOIN_OPEN:  // Received linking request from main unit
          ESP_LOGD(TAG, "Discovery: Found unit type 0x%02X (%s) with ID 0x%02X on network 0x%08X", pResponse->tx_type,
                   pResponse->tx_type == FAN_TYPE_MAIN_UNIT ? "Main" : "?", pResponse->tx_id,
                   pResponse->payload.networkJoinOpen.networkId);

          this->rfComplete();

          (void) memset(this->_txFrame, 0, FAN_FRAMESIZE);  // Clear frame data

          // Found a main unit, so send a join request
          pTxFrame->rx_type = FAN_TYPE_MAIN_UNIT;  // Set type to main unit
          pTxFrame->rx_id = pResponse->tx_id;      // Set ID to the ID of the main unit
          pTxFrame->tx_type = this->config_.fan_my_device_type;
          pTxFrame->tx_id = this->config_.fan_my_device_id;
          pTxFrame->ttl = FAN_TTL;
          pTxFrame->command = FAN_NETWORK_JOIN_REQUEST;  // Request to connect to network
          pTxFrame->parameter_count = sizeof(RfPayloadNetworkJoinOpen);
          // Request to connect to the received network ID
          pTxFrame->payload.networkJoinRequest.networkId = pResponse->payload.networkJoinOpen.networkId;

          // Store for later
          this->config_.fan_networkId = pResponse->payload.networkJoinOpen.networkId;
          this->config_.fan_main_unit_type = pResponse->tx_type;
          this->config_.fan_main_unit_id = pResponse->tx_id;

          // Update address
          rfConfig = this->rf_->getConfig();
          rfConfig.rx_address = pResponse->payload.networkJoinOpen.networkId;
          this->rf_->updateConfig(&rfConfig, NULL);
          this->rf_->writeTxAddress(pResponse->payload.networkJoinOpen.networkId, NULL);

          // Send response frame
          this->startTransmit(this->_txFrame, FAN_TX_RETRIES, [this]() {
            ESP_LOGW(TAG, "Query Timeout");
            this->state_ = StateStartDiscovery;
          });

          this->state_ = StateDiscoveryWaitForJoinResponse;
          break;

        default:
          ESP_LOGD(TAG, "Discovery: Received unknown frame type 0x%02X from ID 0x%02X", pResponse->command,
                   pResponse->tx_id);
          break;
      }
      break;

    case StateDiscoveryWaitForJoinResponse:
      ESP_LOGD(TAG, "DiscoverStateWaitForJoinResponse");
      switch (pResponse->command) {
        case FAN_FRAME_0B:
          if ((pResponse->rx_type == this->config_.fan_my_device_type) &&
              (pResponse->rx_id == this->config_.fan_my_device_id) &&
              (pResponse->tx_type == this->config_.fan_main_unit_type) &&
              (pResponse->tx_id == this->config_.fan_main_unit_id)) {
            ESP_LOGD(TAG, "Discovery: Link successful to unit with ID 0x%02X on network 0x%08X", pResponse->tx_id,
                     this->config_.fan_networkId);

            this->rfComplete();

            (void) memset(this->_txFrame, 0, FAN_FRAMESIZE);  // Clear frame data

            pTxFrame->rx_type = FAN_TYPE_MAIN_UNIT;  // Set type to main unit
            pTxFrame->rx_id = pResponse->tx_id;      // Set ID to the ID of the main unit
            pTxFrame->tx_type = this->config_.fan_my_device_type;
            pTxFrame->tx_id = this->config_.fan_my_device_id;
            pTxFrame->ttl = FAN_TTL;
            pTxFrame->command = FAN_FRAME_0B;  // 0x0B acknowledge link successful
            pTxFrame->parameter_count = 0x00;  // No parameters

            // Send response frame
            this->startTransmit(this->_txFrame, FAN_TX_RETRIES, [this]() {
              ESP_LOGW(TAG, "Query Timeout");
              this->state_ = StateStartDiscovery;
            });

            this->state_ = StateDiscoveryJoinComplete;
          } else {
            ESP_LOGE(TAG, "Discovery: Received unknown link success from ID 0x%02X on network 0x%08X", pResponse->tx_id,
                     this->config_.fan_networkId);
          }
          break;

        default:
          ESP_LOGE(TAG, "Discovery: Received unknown frame type 0x%02X from ID 0x%02X", pResponse->command,
                   pResponse->tx_id);
          break;
      }
      break;

    case StateDiscoveryJoinComplete:
      ESP_LOGD(TAG, "StateDiscoveryJoinComplete");
      switch (pResponse->command) {
        case FAN_TYPE_QUERY_NETWORK:
          if ((pResponse->rx_type == this->config_.fan_main_unit_type) &&
              (pResponse->rx_id == this->config_.fan_main_unit_id) &&
              (pResponse->tx_type == this->config_.fan_main_unit_type) &&
              (pResponse->tx_id == this->config_.fan_main_unit_id)) {
            ESP_LOGD(TAG, "Discovery: received network join success 0x0D");

            this->rfComplete();

            ESP_LOGD(TAG, "Saving pairing config");
            this->pref_.save(&this->config_);

            this->state_ = StateIdle;
          } else {
            ESP_LOGW(TAG, "Unexpected frame join reponse from Type 0x%02X ID 0x%02X", pResponse->tx_type,
                     pResponse->tx_id);
          }
          break;

        default:
          ESP_LOGE(TAG, "Discovery: Received unknown frame type 0x%02X from ID 0x%02X on network 0x%08X",
                   pResponse->command, pResponse->tx_id, this->config_.fan_networkId);
          break;
      }
      break;

    case StateWaitQueryResponse:
      if ((pResponse->rx_type == this->config_.fan_my_device_type) &&  // If type
          (pResponse->rx_id == this->config_.fan_my_device_id)) {      // and id match, it is for us
        switch (pResponse->command) {
          case FAN_TYPE_FAN_SETTINGS:
            ESP_LOGD(TAG, "Received fan settings; speed: 0x%02X voltage: %i timer: %i",
                     pResponse->payload.fanSettings.speed, pResponse->payload.fanSettings.voltage,
                     pResponse->payload.fanSettings.timer);

            this->rfComplete();

            this->state = pResponse->payload.fanSettings.speed > 0;
            this->speed = pResponse->payload.fanSettings.speed;
            this->timer = pResponse->payload.fanSettings.timer;
            this->voltage = pResponse->payload.fanSettings.voltage;
            this->publish_state();

            this->state_ = StateIdle;
            break;

          default:
            ESP_LOGD(TAG, "Received unexpected frame; type 0x%02X from ID 0x%02X", pResponse->command,
                     pResponse->tx_id);
            break;
        }
      } else {
        ESP_LOGD(TAG, "Received frame from unknown device; type 0x%02X from ID 0x%02X type 0x%02X", pResponse->command,
                 pResponse->tx_id, pResponse->tx_type);
      }
      break;

    case StateWaitFilterStatusResponse:
      if ((pResponse->rx_type == this->config_.fan_my_device_type) &&  // If type
          (pResponse->rx_id == this->config_.fan_my_device_id)) {      // and id match, it is for us
        switch (pResponse->command) {
          case FAN_TYPE_FILTER_STATUS_RESPONSE: {
            const RfPayloadFilterStatus* filterStatus =
                reinterpret_cast<const RfPayloadFilterStatus*>(&pResponse->payload);

            ESP_LOGD(TAG, "Received filter status; total hours: %u, filter hours: %u, remaining: %u%%",
                     filterStatus->totalRunHours, filterStatus->filterRunHours,
                     filterStatus->filterPercentRemaining);

            this->rfComplete();

            // Update sensor values if you've added them
            if (this->filter_remaining_sensor_ != nullptr) {
              this->filter_remaining_sensor_->publish_state(filterStatus->filterPercentRemaining);
            }
            if (this->filter_runtime_sensor_ != nullptr) {
              this->filter_runtime_sensor_->publish_state(filterStatus->filterRunHours);
            }

            this->state_ = StateIdle;
            break;
          }

          default:
            ESP_LOGD(TAG, "Received unexpected frame; type 0x%02X from ID 0x%02X",
                     pResponse->command, pResponse->tx_id);
            break;
        }
      } else {
        ESP_LOGD(TAG, "Received frame from unknown device; type 0x%02X from ID 0x%02X type 0x%02X",
                 pResponse->command, pResponse->tx_id, pResponse->tx_type);
      }
      break;

    case StateWaitErrorStatusResponse:
      if ((pResponse->rx_type == this->config_.fan_my_device_type) &&  // If type
          (pResponse->rx_id == this->config_.fan_my_device_id)) {      // and id match, it is for us
        switch (pResponse->command) {
          case FAN_TYPE_ERROR_STATUS_RESPONSE: {
            const RfPayloadErrorStatus* errorStatus =
                reinterpret_cast<const RfPayloadErrorStatus*>(&pResponse->payload);

            ESP_LOGD(TAG, "Received error status; error count: %u", errorStatus->errorCount);

            this->rfComplete();

            // Update error count sensor
            if (this->error_count_sensor_ != nullptr) {
              this->error_count_sensor_->publish_state(errorStatus->errorCount);
            }

            // Build error code string if there are errors
            if (this->error_code_sensor_ != nullptr) {
              if (errorStatus->errorCount > 0) {
                char errorString[32];
                char* p = errorString;
                
                // Format error codes as comma-separated list
                for (int i = 0; i < errorStatus->errorCount && i < 5; i++) {
                  if (i > 0) {
                    *p++ = ',';
                    *p++ = ' ';
                  }
                  p += sprintf(p, "E%u", errorStatus->errorCodes[i]);
                }
                *p = '\0';
                
                this->error_code_sensor_->publish_state(errorString);
              } else {
                this->error_code_sensor_->publish_state("None");
              }
            }

            this->state_ = StateIdle;
            break;
          }

          default:
            ESP_LOGD(TAG, "Received unexpected frame; type 0x%02X from ID 0x%02X",
                     pResponse->command, pResponse->tx_id);
            break;
        }
      } else {
        ESP_LOGD(TAG, "Received frame from unknown device; type 0x%02X from ID 0x%02X type 0x%02X",
                 pResponse->command, pResponse->tx_id, pResponse->tx_type);
      }
      break;

    case StateWaitSetSpeedResponse:
      if ((pResponse->rx_type == this->config_.fan_my_device_type) &&  // If type
          (pResponse->rx_id == this->config_.fan_my_device_id)) {      // and id match, it is for us
        switch (pResponse->command) {
          case FAN_TYPE_FAN_SETTINGS:
            ESP_LOGD(TAG, "Received fan settings; speed: 0x%02X voltage: %i timer: %i",
                     pResponse->payload.fanSettings.speed, pResponse->payload.fanSettings.voltage,
                     pResponse->payload.fanSettings.timer);

            this->rfComplete();

            this->state = pResponse->payload.fanSettings.speed > 0;
            this->speed = pResponse->payload.fanSettings.speed;
            this->timer = pResponse->payload.fanSettings.timer;
            this->voltage = pResponse->payload.fanSettings.voltage;
            this->publish_state();

            (void) memset(this->_txFrame, 0, FAN_FRAMESIZE);  // Clear frame data

            pTxFrame->rx_type = this->config_.fan_main_unit_type;  // Set type to main unit
            pTxFrame->rx_id = this->config_.fan_main_unit_id;      // Set ID to the ID of the main unit
            pTxFrame->tx_type = this->config_.fan_my_device_type;
            pTxFrame->tx_id = this->config_.fan_my_device_id;
            pTxFrame->ttl = FAN_TTL;
            pTxFrame->command = FAN_FRAME_SETSPEED_REPLY;  // 0x0B acknowledge link successful
            pTxFrame->parameter_count = 0x03;              // 3 parameters
            pTxFrame->payload.parameters[0] = 0x54;
            pTxFrame->payload.parameters[1] = 0x03;
            pTxFrame->payload.parameters[2] = 0x20;

            // Send response frame
            this->startTransmit(this->_txFrame, -1, NULL);

            this->state_ = StateWaitSetSpeedConfirm;
            break;

          case FAN_FRAME_SETSPEED_REPLY:
          case FAN_FRAME_SETVOLTAGE_REPLY:
            // this->rfComplete();

            // this->state_ = StateIdle;
            break;

          default:
            ESP_LOGD(TAG, "Received unexpected frame; type 0x%02X from ID 0x%02X", pResponse->command,
                     pResponse->tx_id);
            break;
        }
      } else {
        ESP_LOGD(TAG, "Received frame from unknown device; type 0x%02X from ID 0x%02X type 0x%02X", pResponse->command,
                 pResponse->tx_id, pResponse->tx_type);
      }
      break;

    default:
      ESP_LOGD(TAG, "Received frame from unknown device in unknown state; type 0x%02X from ID 0x%02X type 0x%02X",
               pResponse->command, pResponse->tx_id, pResponse->tx_type);
      break;
  }
}

static uint8_t minmax(const uint8_t value, const uint8_t min, const uint8_t max) {
  if (value <= min) {
    return min;
  } else if (value >= max) {
    return max;
  } else {
    return value;
  }
}

uint8_t ZehnderRF::createDeviceID(void) {
  uint8_t random = (uint8_t) random_uint32();
  // Generate random device_id; don't use 0x00 and 0xFF

  // TODO: there's a 1 in 255 chance that the generated ID matches the ID of the main unit. Decide how to deal
  // withthis (some sort of ping discovery?)

  return minmax(random, 1, 0xFE);
}

void ZehnderRF::queryDevice(void) {
  RfFrame *const pFrame = (RfFrame *) this->_txFrame;  // frame helper

  ESP_LOGD(TAG, "Query device");

  (void) memset(this->_txFrame, 0, FAN_FRAMESIZE);  // Clear frame data

  pFrame->rx_type = this->config_.fan_main_unit_type;  // Main
  pFrame->rx_id = this->config_.fan_main_unit_id;
  pFrame->tx_type = this->config_.fan_my_device_type;
  pFrame->tx_id = this->config_.fan_my_device_id;
  pFrame->ttl = FAN_TTL;
  pFrame->command = FAN_TYPE_QUERY_DEVICE;  // Request info
  pFrame->parameter_count = 0x00;           // No params

  this->startTransmit(this->_txFrame, FAN_TX_RETRIES, [this]() {
    ESP_LOGW(TAG, "Query timeout");
    this->state_ = StateIdle;
  });

  this->lastFanQuery_ = millis();  // Update time
  this->state_ = StateWaitQueryResponse;
}

void ZehnderRF::setSpeed(const uint8_t paramSpeed, const uint8_t paramTimer) {
  RfFrame *const pFrame = (RfFrame *) this->_txFrame;  // frame helper
  uint8_t speed = paramSpeed;
  uint8_t timer = paramTimer;

  if (speed > this->speed_count_) {
    ESP_LOGW(TAG, "Requested speed too high (%u)", speed);
    speed = this->speed_count_;
  }

  ESP_LOGD(TAG, "Set speed: 0x%02X; Timer %u minutes", speed, timer);

  if (this->state_ == StateIdle) {
    (void) memset(this->_txFrame, 0, FAN_FRAMESIZE);  // Clear frame data

    // Build frame
    pFrame->rx_type = this->config_.fan_main_unit_type;
    pFrame->rx_id = 0x00;  // Broadcast
    pFrame->tx_type = this->config_.fan_my_device_type;
    pFrame->tx_id = this->config_.fan_my_device_id;
    pFrame->ttl = FAN_TTL;

    if (timer == 0) {
      pFrame->command = FAN_FRAME_SETSPEED;
      pFrame->parameter_count = sizeof(RfPayloadFanSetSpeed);
      pFrame->payload.setSpeed.speed = speed;
    } else {
      pFrame->command = FAN_FRAME_SETTIMER;
      pFrame->parameter_count = sizeof(RfPayloadFanSetTimer);
      pFrame->payload.setTimer.speed = speed;
      pFrame->payload.setTimer.timer = timer;
    }

    this->startTransmit(this->_txFrame, FAN_TX_RETRIES, [this]() {
      ESP_LOGW(TAG, "Set speed timeout");
      this->state_ = StateIdle;
    });

    newSetting = false;
    this->state_ = StateWaitSetSpeedResponse;
  } else {
    ESP_LOGD(TAG, "Invalid state, I'm trying later again");
    newSpeed = speed;
    newTimer = timer;
    newSetting = true;
  }
}

void ZehnderRF::discoveryStart(const uint8_t deviceId) {
  RfFrame *const pFrame = (RfFrame *) this->_txFrame;  // frame helper
  nrf905::Config rfConfig;

  ESP_LOGD(TAG, "Start discovery with ID %u", deviceId);

  this->config_.fan_my_device_type = FAN_TYPE_REMOTE_CONTROL;
  this->config_.fan_my_device_id = deviceId;

  // Reset radio to ensure clean state
  this->rf_->setMode(nrf905::Idle);

  // Set RX and TX address to network link ID
  rfConfig = this->rf_->getConfig();
  rfConfig.rx_address = NETWORK_LINK_ID;
  this->rf_->updateConfig(&rfConfig, NULL);
  this->rf_->writeTxAddress(NETWORK_LINK_ID, NULL);
  
  // Ensure we're in receive mode before transmitting
  this->rf_->setMode(nrf905::Receive);

  // Build frame
  (void) memset(this->_txFrame, 0, FAN_FRAMESIZE);  // Clear frame data

  // Set payload, available for linking
  pFrame->rx_type = 0x04;
  pFrame->rx_id = 0x00;
  pFrame->tx_type = this->config_.fan_my_device_type;
  pFrame->tx_id = this->config_.fan_my_device_id;
  pFrame->ttl = FAN_TTL;
  pFrame->command = FAN_NETWORK_JOIN_ACK;
  pFrame->parameter_count = sizeof(RfPayloadNetworkJoinAck);
  pFrame->payload.networkJoinAck.networkId = NETWORK_LINK_ID;

  this->startTransmit(this->_txFrame, FAN_TX_RETRIES * 2, [this]() {  // Double the retries for discovery
    ESP_LOGW(TAG, "Start discovery timeout");
    this->state_ = StateStartDiscovery;
  });

  // Update state
  this->state_ = StateDiscoveryWaitForLinkRequest;
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

    // Put radio in idle mode to prepare for transmission
    this->rf_->setMode(nrf905::Idle);

    // Write data to RF
    ESP_LOGD(TAG, "Writing payload to RF transmitter");
    this->rf_->writeTxPayload(pData, FAN_FRAMESIZE);
    
    this->rfState_ = RfStateWaitAirwayFree;
    this->airwayFreeWaitTime_ = millis();
  }

  return result;
}

void ZehnderRF::rfComplete(void) {
  this->retries_ = -1;  // Disable this->retries_
  this->rfState_ = RfStateIdle;
}

void ZehnderRF::rfHandler(void) {
  static uint32_t next_airway_check = 0;
  static uint32_t next_retry_time = 0;

  switch (this->rfState_) {
    case RfStateIdle:
      break;

    case RfStateWaitAirwayFree:
      // Check airway periodically without blocking
      if (millis() >= next_airway_check) {
         if ((millis() - this->airwayFreeWaitTime_) > 5000) { // Timeout check
           ESP_LOGW(TAG, "Airway too busy, giving up");
           this->rfState_ = RfStateIdle;

           if (this->onReceiveTimeout_ != NULL) {
             this->onReceiveTimeout_();
           }
           // Reset state immediately
           break;
         } 

        if (this->rf_->airwayBusy() == false) {
          ESP_LOGD(TAG, "Airway free, starting transmission");
          this->rf_->startTx(FAN_TX_FRAMES, nrf905::Receive);  // After transmit, wait for response
          this->rfState_ = RfStateTxBusy;
        } else {
          // If airway is busy, schedule the next check shortly
          ESP_LOGV(TAG, "Airway busy, waiting...");
          next_airway_check = millis() + 50; // Check again in 50ms
          yield(); // Allow other tasks to run
        }
      }
      break;

    case RfStateTxBusy:
      // Waiting for TX completion handled by onTxReady callback
      break;

    case RfStateRxWait:
      if ((this->retries_ >= 0) && (millis() >= next_retry_time)) { 
        // Check if actual timeout occurred based on msgSendTime_
        if ((millis() - this->msgSendTime_) > FAN_REPLY_TIMEOUT) {
           ESP_LOGD(TAG, "Receive timeout");

           if (this->retries_ > 0) {
             --this->retries_;
             ESP_LOGD(TAG, "No data received, retry again (left: %u)", this->retries_);

             // Schedule next attempt after a short non-blocking delay
             this->rfState_ = RfStateWaitAirwayFree; // Go back to checking airway before retry
             this->airwayFreeWaitTime_ = millis(); // Reset airway timer
             next_airway_check = millis() + 150 + (random_uint32() % 200); // Wait 150-350ms before checking airway for retry

           } else { // retries_ == 0
             // Oh oh, ran out of options
             ESP_LOGD(TAG, "No messages received, giving up now...");
             if (this->onReceiveTimeout_ != NULL) {
               this->onReceiveTimeout_();
             }

             // Back to idle
             this->rfState_ = RfStateIdle;
           }
        } else {
           // Not timed out yet, schedule next check
           next_retry_time = millis() + 50; // Check again in 50ms
           yield(); // Allow other tasks to run
        }
      }
      break;

    default:
      break;
  }
}

}  // namespace zehnder
}  // namespace esphome
