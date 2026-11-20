/*
 * Apres Light Switch Module using SensESP V3
 * Copyright (c) 2026 Jason Greenwood
 * [https://www.youtube.com/@ApresSail](https://www.youtube.com/@ApresSail)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
// Apres Light Switch Module using SensESP V3.
// Software version 3.1.0 - 20261120
// Included in this version:
// 1. 4-channel relay module with feedback and Signal K integration. Each channel can be controlled via Signal K and provides feedback on its state.
// 2. USB serial command interface for manual control and debugging.
// 3. Configuration options for each relay channel, allowing users to specify Signal K paths and other settings.
//

#include <Arduino.h>

#include "sensesp.h"
#include "sensesp_app_builder.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/signalk/signalk_put_request_listener.h"
#include "sensesp/system/lambda_consumer.h"

using namespace sensesp;

constexpr uint32_t SAMPLE_MS = 20;
constexpr uint32_t DEBOUNCE_MS = 100;
constexpr uint32_t SETTLE_MS = 400;
constexpr uint32_t VERIFY_TIMEOUT_MS = 2000;
constexpr uint32_t REPORT_MS = 5000;

class LightChannel {
 public:
  LightChannel(const char* name, uint8_t relay_pin,
               uint8_t feedback_pin)
      : name_(name),
        relay_pin_(relay_pin),
        feedback_pin_(feedback_pin) {}

  void begin() {
    // Establish a known relay output level.
    // This does NOT imply that the light itself is off.
    digitalWrite(relay_pin_, LOW);
    pinMode(relay_pin_, OUTPUT);
    relay_level_ = false;

    // External 10k pull-up to 3.3 V is fitted.
    pinMode(feedback_pin_, INPUT);

    candidate_on_ = read_light_on();
    candidate_since_ = millis();

    String base = String("electrical.switches.") + name_;

    // Fixed paths keep the listener and publisher aligned.
    state_output_ = new SKOutputBool(base + ".state");
    fault_output_ = new SKOutputBool(base + ".feedbackFault");

    auto* listener =
        new BoolSKPutRequestListener(base + ".state");

    listener->connect_to(
        new LambdaConsumer<bool>(
            [this](bool desired_on) {
              request_state(desired_on);
            }));

    event_loop()->onRepeat(
        SAMPLE_MS, [this]() { update(); });
  }

 private:
  bool read_light_on() const {
    return digitalRead(feedback_pin_) == LOW;
  }

  void publish_state(uint32_t now) {
    state_output_->set(measured_on_);
    last_report_ = now;
  }

  void request_state(bool desired_on) {
    uint32_t now = millis();

    // Do not operate using uninitialized or unsettled feedback.
    if (!feedback_valid_ ||
        candidate_on_ != measured_on_) {
      Serial.printf("%s: feedback not ready; request ignored\n",
                    name_);
      return;
    }

    // Prevent duplicate requests from reversing the relay twice.
    if (pending_ ||
        (has_switched_ &&
         uint32_t(now - switched_at_) < SETTLE_MS)) {
      Serial.printf("%s: busy; request ignored\n", name_);
      return;
    }

    if (desired_on == measured_on_) {
      publish_state(now);
      return;
    }

    requested_on_ = desired_on;
    pending_ = true;
    fault_ = false;
    fault_output_->set(false);

    relay_level_ = !relay_level_;
    digitalWrite(relay_pin_, relay_level_ ? HIGH : LOW);

    switched_at_ = now;
    has_switched_ = true;

    Serial.printf("%s: requested %s; relay reversed\n",
                  name_, desired_on ? "ON" : "OFF");
  }

  void update() {
    uint32_t now = millis();
    bool raw_on = read_light_on();

    if (raw_on != candidate_on_) {
      candidate_on_ = raw_on;
      candidate_since_ = now;
    }

    // Accept a new feedback value only after it stays stable.
    if (uint32_t(now - candidate_since_) >= DEBOUNCE_MS) {
      if (!feedback_valid_ ||
          measured_on_ != candidate_on_) {
        measured_on_ = candidate_on_;
        feedback_valid_ = true;
        publish_state(now);

        Serial.printf("%s: measured %s\n",
                      name_, measured_on_ ? "ON" : "OFF");
      }
    }

    if (pending_ &&
        uint32_t(now - switched_at_) >= SETTLE_MS) {
      bool stable = feedback_valid_ &&
                    candidate_on_ == measured_on_ &&
                    uint32_t(now - candidate_since_) >=
                        DEBOUNCE_MS;

      if (stable && measured_on_ == requested_on_) {
        pending_ = false;
        Serial.printf("%s: request verified\n", name_);
      } else if (uint32_t(now - switched_at_) >=
                 VERIFY_TIMEOUT_MS) {
        pending_ = false;
        fault_ = true;
        fault_output_->set(true);

        Serial.printf(
            "%s: feedback timeout; no automatic retry\n",
            name_);
      }
    }

    if (feedback_valid_ &&
        uint32_t(now - last_report_) >= REPORT_MS) {
      publish_state(now);
      fault_output_->set(fault_);
    }
  }

  const char* name_;
  uint8_t relay_pin_;
  uint8_t feedback_pin_;

  bool relay_level_ = false;
  bool candidate_on_ = false;
  bool measured_on_ = false;
  bool feedback_valid_ = false;
  bool requested_on_ = false;
  bool pending_ = false;
  bool has_switched_ = false;
  bool fault_ = false;

  uint32_t candidate_since_ = 0;
  uint32_t switched_at_ = 0;
  uint32_t last_report_ = 0;

  SKOutputBool* state_output_ = nullptr;
  SKOutputBool* fault_output_ = nullptr;
};

// Existing project pin assignments: verify before installation.
LightChannel anchor("anchorLight", 12, 25);
LightChannel deck("deckLight", 4, 26);
LightChannel steaming("steamingLight", 16, 0);
LightChannel running("runningLights", 17, 14);

void setup() {
  SetupSerialDebug(115200);

  SensESPAppBuilder builder;
  sensesp_app = (&builder)
                    ->set_hostname("Apres-Light-Switch-Module")
                    ->get_app();

  anchor.begin();
  deck.begin();
  steaming.begin();
  running.begin();
}

void loop() {
  event_loop()->tick();
}