#include "mature_blinds.h"
#include "esphome/core/log.h"

namespace esphome {
namespace mature_blinds {
using namespace esphome::cover;
using namespace esphome::esphome_serial;

static const char *const TAG = "mature_blinds";
FastAccelStepperEngine *MatureBlinds::engine_ = nullptr;
void IRAM_ATTR MatureBlinds::diag_isr_(MatureBlinds *arg) {
  arg->stall_detected_ = true;
  arg->stepper_->forceStopAndNewPosition(-1 * arg->homing_gap_);
}

void MatureBlinds::setup() {
  if (en_pin_)
    pinMode(en_pin_, OUTPUT);

  pinMode(dir_pin_, OUTPUT);
  pinMode(step_pin_, OUTPUT);
  // pinMode(diag_pin_, INPUT_PULLUP);
  diag_pin_->setup();

  init_driver_();
  init_stepper_();

  auto restore = this->restore_state_();
  if (restore.has_value()) {
    int32_t steps = get_current_steps(restore->position);
    stepper_->setCurrentPosition(steps);
    restore->apply(this);
  } else {
    // Do homing
  }
  setup_done_ = true;
}

void MatureBlinds::loop() {
  unsigned long ms = millis();

  if (is_moving_) {
    if (!stepper_->isRunning()) {
      is_moving_ = false;
      this->position = get_current_position();
      this->current_operation = CoverOperation::COVER_OPERATION_IDLE;
      this->publish_state();
      on_after_stop_callback_.call();

      ESP_LOGI(TAG, "Position: %.2f (%ld)", this->position, stepper_->getCurrentPosition());
    } else if (ms - last_publish_time_ > 1000 || ms < last_publish_time_) {
      last_publish_time_ = ms;
      this->publish_state();
    }
  }

  if (is_homing_) {
    if (stall_detected_) {
      ESP_LOGI(TAG, "STALL DETECTED");
      stall_detected_ = false;
      end_homing_();
    } else if (ms - last_publish_time_ > 1000 || ms < last_publish_time_) {
      last_publish_time_ = ms;
      this->publish_state();
    }
  }
}

void MatureBlinds::dump_config() {
  ESP_LOGCONFIG(TAG, "Mature Blinds (%s):", get_name());
  if (en_pin_)
    ESP_LOGCONFIG(TAG, "  en_pin: %d", en_pin_);
  ESP_LOGCONFIG(TAG, "  dir_pin: %d", dir_pin_);
  ESP_LOGCONFIG(TAG, "  step_pin: %d", step_pin_);
  ESP_LOGCONFIG(TAG, "  diag_pin: %d", diag_pin_->get_pin());
  ESP_LOGCONFIG(TAG, "  address: %d", address_);
  ESP_LOGCONFIG(TAG, "  r_sense: %.3f", r_sense_);
  ESP_LOGCONFIG(TAG, "  rms_current: %u", rms_current_);
  ESP_LOGCONFIG(TAG, "  acceleration: %d", acceleration_);
  ESP_LOGCONFIG(TAG, "  speed_in_us: %u", speed_);
  ESP_LOGCONFIG(TAG, "  stall_value: %u", stall_value_);
  ESP_LOGCONFIG(TAG, "  test connection: %d", driver_->test_connection());
  ESP_LOGCONFIG(TAG, "  setup: %d", setup_done_);
}

float MatureBlinds::get_setup_priority() const { return setup_priority::DATA; }

cover::CoverTraits MatureBlinds::get_traits() {
  auto traits = CoverTraits();
  traits.set_is_assumed_state(false);
  traits.set_supports_position(true);
  traits.set_supports_tilt(false);
  traits.set_supports_stop(true);
  traits.set_supports_toggle(false);
  return traits;
}

void MatureBlinds::home() { start_homing_(); }

void MatureBlinds::move_down(int32_t value) {
  ESP_LOGI(TAG, "MOVE DOWN: %ld", value);
  is_moving_ = true;
  on_before_start_callback_.call();
  stepper_->move(value);
}

void MatureBlinds::control(const CoverCall &call) {
  if (call.get_position().has_value()) {
    float pos = *call.get_position();
    int32_t target = full_distance_ * (1 - pos);

    is_moving_ = true;
    on_before_start_callback_.call();
    stepper_->moveTo(target);
    ESP_LOGD(TAG, "(%s) is moving to: %.2f (%ld)", get_name(), pos, target);

    this->current_operation = get_operation(pos);
    this->publish_state();
  }

  if (call.get_stop()) {
    stepper_->forceStop();
    this->position = get_current_position();
    this->publish_state();
  }
}

float MatureBlinds::get_current_position() {
  return 1 - static_cast<float>(stepper_->getCurrentPosition()) / full_distance_;
}

int32_t MatureBlinds::get_current_steps(float pos) { return (1 - pos) * full_distance_; }

CoverOperation MatureBlinds::get_operation(float pos) {
  if (this->position == pos)
    return CoverOperation::COVER_OPERATION_IDLE;

  return pos > this->position ? CoverOperation::COVER_OPERATION_OPENING : CoverOperation::COVER_OPERATION_CLOSING;
}

void MatureBlinds::start_homing_() {
  ESP_LOGD(TAG, "Start homing");
  on_before_start_callback_.call();
  
  stepper_->setCurrentPosition(999999);
  stepper_->setSpeedInUs(speed_ * 2);
  this->current_operation = CoverOperation::COVER_OPERATION_OPENING;
  stepper_->moveTo(0);

  delay(1000);
  is_homing_ = true;
  stall_detected_ = false;
  last_publish_time_ = 0;
  this->diag_pin_->attach_interrupt(this->diag_isr_, this, gpio::InterruptType::INTERRUPT_RISING_EDGE);
}

void MatureBlinds::end_homing_() {
  this->diag_pin_->detach_interrupt();
  is_homing_ = false;
  ESP_LOGD(TAG, "Stop homing, position: %ld", stepper_->getCurrentPosition());

  stepper_->setSpeedInUs(speed_);
  stepper_->moveTo(0, true);
  ESP_LOGD(TAG, "Position: %ld", stepper_->getCurrentPosition());

  this->current_operation = CoverOperation::COVER_OPERATION_IDLE;
  this->position = get_current_position();
  this->publish_state();

  on_after_stop_callback_.call();
}

void MatureBlinds::init_driver_() {
  auto serail = new EsphomeSerial(this);
  driver_ = new TMC2209Stepper(serail, this->r_sense_, this->address_);
  driver_->begin();

  driver_->toff(2);
  driver_->blank_time(24);
  driver_->rms_current(rms_current_);
  driver_->microsteps(32);
  driver_->pwm_autoscale(true);
  driver_->TCOOLTHRS(0xFFFFF);
  driver_->SGTHRS(stall_value_);
}

void MatureBlinds::init_stepper_() {
  if (!MatureBlinds::engine_) {
    MatureBlinds::engine_ = new FastAccelStepperEngine();
    MatureBlinds::engine_->init();
  }

  stepper_ = MatureBlinds::engine_->stepperConnectToPin(step_pin_);
  stepper_->setDirectionPin(dir_pin_, !invert_rotation_);
  stepper_->setSpeedInUs(speed_);
  stepper_->setAcceleration(acceleration_);

  if (en_pin_) {
    stepper_->setEnablePin(en_pin_);
    stepper_->setAutoEnable(true);
  }
}

}  // namespace mature_blinds
}  // namespace esphome
