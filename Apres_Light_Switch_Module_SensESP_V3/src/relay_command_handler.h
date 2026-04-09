#include "Arduino.h"
#include "sensesp/controllers/smart_switch_controller.h"
#include "sensesp/system/valueproducer.h"
#include "sensesp/system/valueconsumer.h"

using namespace std;

using namespace sensesp;

class RelayCommandHandler : public StringProducer, public StringConsumer {
    private:
        SmartSwitchController* controller;
        String _state = "idle";
        bool _inAction = false;

        void repeat_function() 
        {
            this->notify();
        }

        void set_state(const String& new_state) 
        {
            _state = new_state;
            this->emit(_state);
        }

        bool start_action(String action) 
        {
            if(_inAction) return false; // already in action
            _inAction = true;
            set_state(action);
            _inAction = false;

            return true;
        }

        bool stop_action() 
        {
            if(!_inAction) return false; // not in action
            _inAction = false;
            controller->emit(false);
            set_state("idle");
            return true;
        }

    public:
        RelayCommandHandler(SmartSwitchController* ctrl, int interval = 10000) : controller(ctrl) 
        {
            emit(_state);

            event_loop()->onRepeat(interval, [this]() { this->repeat_function(); });
        }

        void set(const String& new_value) override 
        {
            if(start_action(new_value) == false) 
            {
                // already in action
                return;
            }

            if (new_value == "toggle") {
                controller->emit(!controller->get());
                _state = controller->get() ? "on" : "off";
                stop_action();
            } else if (new_value == "on") {
                controller->emit(true);
                _state = "on";
                stop_action();
            } else if (new_value == "off") {
                controller->emit(false);
                _state = "off";
                stop_action();
            }
            else if(new_value.startsWith("click")) //click = HIGH => 1 second delay => LOW
            {                                      //click:<timeout> ms
                int click_timeout = 1000;

                if(new_value.startsWith("click:")) //click:<timeout in seconds>
                {
                    click_timeout = new_value.substring(6).toInt() * 1000;
                }
                //new_value.substring(6).toInt();
                controller->emit(true);
                _state = "clicked " + String(click_timeout/1000) + "s";
                event_loop()->onDelay(click_timeout, [this]() { this->controller->emit(false); this->stop_action(); });                
            }
            else 
            {
                // Invalid command, ignore
                stop_action();
                return;
            }
        }

        SmartSwitchController* getController() { return controller; }
};