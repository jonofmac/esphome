#include "hx711.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace hx711 {

static const char *const TAG = "hx711";

void HX711Sensor::setup() {
  ESP_LOGCONFIG(TAG, "Setting up HX711 '%s'...", this->name_.c_str());
  this->sck_pin_->setup();
  this->dout_pin_->setup();
  this->sck_pin_->digital_write(false);

  // Read sensor once without publishing to set the gain
  this->read_sensor_(nullptr);
}

void HX711Sensor::dump_config() {
  LOG_SENSOR("", "HX711", this);
  LOG_PIN("  DOUT Pin: ", this->dout_pin_);
  LOG_PIN("  SCK Pin: ", this->sck_pin_);
  LOG_UPDATE_INTERVAL(this);
}
float HX711Sensor::get_setup_priority() const { return setup_priority::DATA; }
void HX711Sensor::update() {
  uint32_t result;
  if (this->read_sensor_(&result)) {
    int32_t value = static_cast<int32_t>(result);
    ESP_LOGD(TAG, "'%s': Got raw value %" PRId32, this->name_.c_str(), value);
    this->publish_state(value);
  }
}
bool HX711Sensor::read_sensor_(uint32_t *result) {
  if (this->dout_pin_->digital_read()) {
    ESP_LOGW(TAG, "HX711 is not ready for new measurements yet!");
    this->status_set_warning();
    return false;
  }

  this->status_clear_warning();
  uint32_t data = 0;
  uint8_t sample_error = 0;

  {
    InterruptLock lock;
    for (uint8_t i = 0; i < 24; i++) {
      this->sck_pin_->digital_write(true);
      delayMicroseconds(1);
      uint8_t samples = this->dout_pin_->digital_read();
      delayMicroseconds(1);
      samples += this->dout_pin_->digital_read();
      delayMicroseconds(1);
      samples += this->dout_pin_->digital_read();
      // If at least 2 of the 3 samples are high, then we assume high state on pin
      if (samples >= 2)
        data |= uint32_t(1) << (23 - i);
      if ((samples == 1) || (samples == 2))
        sample_error++;
      this->sck_pin_->digital_write(false);
      delayMicroseconds(3);
    }

    // Cycle clock pin for gain setting
    for (uint8_t i = 0; i < this->gain_; i++) {
      this->sck_pin_->digital_write(true);
      delayMicroseconds(3);
      this->sck_pin_->digital_write(false);
      delayMicroseconds(3);
    }
  }

  if (data & 0x800000ULL) {
    data |= 0xFF000000ULL;
  }

  if (sample_error)
    ESP_LOGW(TAG, "HX711 measurement had some detected noise!");

  if (result != nullptr)
    *result = data;
  return true;
}

}  // namespace hx711
}  // namespace esphome
