#pragma once

#include <array>

#include "esphome/core/component.h"
#include "esphome/core/automation.h"
#include "esphome/components/cover/cover.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#include "TMCStepper.h"
#include "FastAccelStepper.h"
#include "esphome_serial.h"

namespace esphome {
namespace mature_blinds {

class MatureBlinds : public cover::Cover, public uart::UARTDevice, public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override;
  cover::CoverTraits get_traits() override;

  void home();
  void move_down(int32_t value);

  void set_r_sense(float v) { r_sense_ = v; };
  void set_address(uint8_t v) { address_ = v; };
  void set_en_pin(uint8_t v) { en_pin_ = v; };
  void set_dir_pin(uint8_t v) { dir_pin_ = v; };
  void set_step_pin(uint8_t v) { step_pin_ = v; };
  void set_diag_pin(InternalGPIOPin *pin) { diag_pin_ = pin; };
  void set_rms_current(uint16_t v) { rms_current_ = v; };
  void set_stall_value(uint8_t v) { stall_value_ = v; };
  void set_speed(uint32_t v) { speed_ = v; };
  void set_acceleration(int32_t v) { acceleration_ = v; };
  void set_full_distance(uint32_t v) { full_distance_ = v; };
  void set_homing_gap(uint32_t v) { homing_gap_ = v; };
  void set_invert_rotation(bool v) { invert_rotation_ = v; };

  void add_on_before_start_callback(std::function<void()> callback) {
    this->on_before_start_callback_.add(std::move(callback));
  }

  void add_on_after_stop_callback(std::function<void()> callback) {
    this->on_after_stop_callback_.add(std::move(callback));
  }

 protected:
  void control(const cover::CoverCall &call) override;

 private:
  float get_current_position();
  int32_t get_current_steps(float pos);
  cover::CoverOperation get_operation(float pos);
  void start_homing_();
  void end_homing_();
  static void diag_isr_(MatureBlinds *arg);
  void init_driver_();
  void init_stepper_();

  TMC2209Stepper *driver_{nullptr};
  static FastAccelStepperEngine *engine_;
  FastAccelStepper *stepper_{nullptr};

  float r_sense_{0};
  uint8_t address_{0};
  uint8_t en_pin_{0};
  uint8_t dir_pin_{0};
  uint8_t step_pin_{0};
  InternalGPIOPin *diag_pin_;
  uint16_t rms_current_{0};
  uint8_t stall_value_{0};
  uint32_t speed_{0};
  int32_t acceleration_{0};
  uint32_t full_distance_{0};
  uint32_t homing_gap_{0};
  bool invert_rotation_{false};

  bool setup_done_{false};
  bool is_moving_{false};
  uint32_t last_publish_time_{0};
  bool is_homing_{false};
  volatile bool stall_detected_{false};

  CallbackManager<void()> on_before_start_callback_;
  CallbackManager<void()> on_after_stop_callback_;
};

class BeforeStartTrigger : public Trigger<> {
 public:
  explicit BeforeStartTrigger(MatureBlinds *parent) {
    parent->add_on_before_start_callback([this]() { this->trigger(); });
  }
};
class AfterStopTrigger : public Trigger<> {
 public:
  explicit AfterStopTrigger(MatureBlinds *parent) {
    parent->add_on_after_stop_callback([this]() { this->trigger(); });
  }
};

template<typename... Ts> class HomeAction : public Action<Ts...> {
 public:
  HomeAction(MatureBlinds *parent) : parent_(parent) {}

  void play(const Ts &...x) override { this->parent_->home(); }

 protected:
  MatureBlinds *parent_;
};

template<typename... Ts> class MoveDownAction : public Action<Ts...> {
 public:
  MoveDownAction(MatureBlinds *parent) : parent_(parent) {}
  TEMPLATABLE_VALUE(int32_t, value)

  void play(const Ts &...x) override {
    auto value = this->value_.value(x...);
    this->parent_->move_down(value);
  }

 protected:
  MatureBlinds *parent_;
};

}  // namespace mature_blinds
}  // namespace esphome
