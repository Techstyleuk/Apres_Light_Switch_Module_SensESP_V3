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
// Software version 3.0.0 - 20260408
// Included in this version:
// 1. 4-channel relay module with feedback and Signal K integration. Each channel can be controlled via Signal K and provides feedback on its state.
// 2. USB serial command interface for manual control and debugging.
// 3. Configuration options for each relay channel, allowing users to specify Signal K paths and other settings.
//

#include <memory>

#include "sensesp.h"
#include "sensesp/controllers/smart_switch_controller.h"
#include "sensesp/sensors/digital_input.h"
#include "sensesp/sensors/digital_output.h"
#include "sensesp/sensors/sensor.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/signalk/signalk_put_request_listener.h"
#include "sensesp/system/lambda_consumer.h"
#include "sensesp/transforms/repeat.h"
#include "sensesp_app_builder.h"
#include "USB.h"
#include "relay_command_handler.h"

using namespace sensesp;

const uint8_t kRelayPin1 = 25;     // D2 on Firebeetle, GPIO 25 is used for relay control.
const uint8_t kRelayPin2 = 26;     // D3 on Firebeetle, GPIO 26 is used for relay control.
const uint8_t kRelayPin3 = 0;      // D5 on Firebeetle, GPIO 0 is used for relay control.
const uint8_t kRelayPin4 = 14;     // D6 on Firebeetle, GPIO 14 is used for relay control.

const uint8_t kFeedbackPin1 = 17;  // D10 on Firebeetle, GPIO 17 is used for relay feedback.
const uint8_t kFeedbackPin2 = 16;  // D11 on Firebeetle, GPIO 16 is used for relay feedback.
const uint8_t kFeedbackPin3 = 4;   // D12 on Firebeetle, GPIO 4 is used for relay feedback.
const uint8_t kFeedbackPin4 = 12;  // D13 on Firebeetle, GPIO 12 is used for relay feedback.

struct RelayChannel {
  uint8_t relay_pin;
  uint8_t feedback_pin;
  String name;
  String relay_config_path;
};

RelayCommandHandler* initialize_relay_channel(const RelayChannel& ch) {
  pinMode(ch.relay_pin, OUTPUT);
  digitalWrite(ch.relay_pin, LOW);

  auto* load_switch = new DigitalOutput(ch.relay_pin);

  String sk_state_path = "electrical.switches." + ch.name + ".state";
  String sk_command_path = "electrical.switches." + ch.name + ".command";
  String sk_feedback_path = "electrical.switches." + ch.name + ".feedback";

  auto* controller = new SmartSwitchController(true);
  controller->connect_to(load_switch);

  auto* skMetadata = new SKMetadata("",
                                "",
                                "",
                                "",
                                -1.0f,
                                true);
  auto* feedback = new DigitalInputState(ch.feedback_pin, INPUT_PULLUP, 250);
  feedback->connect_to(new SKOutputBool(sk_state_path, ch.relay_config_path, skMetadata));

  auto* relay_command_handler = new RelayCommandHandler(controller);

  auto* sk_command_listener = new StringSKPutRequestListener(sk_command_path);
  sk_command_listener->connect_to(relay_command_handler);

  relay_command_handler->connect_to(
      new SKOutputString(sk_command_path, ch.relay_config_path));

  return relay_command_handler;
}

void setup() {
  SetupSerialDebug(115200);

  SensESPAppBuilder builder;
  sensesp_app = (&builder)
                    // Set a custom hostname for the app.
                    ->set_hostname("Apres-Light-Switch-Module")
                    ->enable_ota("thisisfine")
                    ->get_app();

  RelayChannel relays[] = {
      {kRelayPin1, kFeedbackPin1, "anchorLight", "sensesp-relay1"},
      {kRelayPin2, kFeedbackPin2, "runningLights", "sensesp-relay2"},
      {kRelayPin3, kFeedbackPin3, "steamingLight", "sensesp-relay3"},
      {kRelayPin4, kFeedbackPin4, "deckLight", "sensesp-relay4"},
  };

  for (auto& ch : relays) {
    initialize_relay_channel(ch);
  }

  while (true) {
    loop();
  }
}

void loop() { event_loop()->tick(); }
