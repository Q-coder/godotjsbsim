#ifndef JSBGODOT_H
#define JSBGODOT_H

#include "FGFDMExec.h"

#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/classes/input_event.hpp>
#include <godot_cpp/classes/input_event_joypad_motion.hpp>
#include <godot_cpp/classes/input_event_joypad_button.hpp>

namespace godot
{

    class JSBGodot : public Node3D
    {
        GDCLASS(JSBGodot, Node3D);

    private:
        JSBSim::FGFDMExec *FDMExec;
        bool do_scripted;

        float input_pitch = 0.0;
        float input_roll = 0.0;
        float input_rudder = 0.0;
        float input_throttle = 0.0;
        float input_aileron = 0.0f;  // Aileron input (-1.0 to 1.0)
        float input_elevator = 0.0f; // Elevator input (-1.0 to 1.0)
        float elevator_trim = 0.0f;  // Elevator trim (-1.0 to 1.0)
        float trigger_left_value = 0.0f;   // LT trigger value for throttle decrease
        float trigger_right_value = 0.0f;  // RT trigger value for throttle increase
        bool trim_up_held = false;   // Y button held for trim up
        bool trim_down_held = false; // X button held for trim down
        double altitude_ft = 0.0;
        double heading_deg = 0.0;
        float flaps = 0.0f; // Flaps input (0.0 to 1.0)
        // Existing control input variables...
        float input_brake = 0.0f; // Brake input (0.0 to 1.0)

        double airspeed_knots;
        double vertical_speed_fpm;

        void copy_inputs_to_JSBSim();
        void copy_outputs_from_JSBSim();

    public:
        static void _bind_methods();

        JSBGodot();
        ~JSBGodot();

        void _ready() override;
        void _process(double delta) override;
        void _input(const Ref<InputEvent> event); // Using Ref<InputEvent> requires full inclusion
        void _physics_process(const real_t delta);
        void initialise();

        void set_input_pitch(float value);
        float get_input_pitch() const;

        void set_input_roll(float value);
        float get_input_roll() const;

        void set_input_rudder(float value);
        float get_input_rudder() const;

        void set_input_throttle(float value);
        float get_input_throttle() const;

        void set_input_elevator(float value);
        float get_input_elevator() const;
        void set_input_aileron(float value);
        float get_input_aileron() const;

        double get_airspeed_knots() const;
        double get_vertical_speed_fpm() const;
        double get_altitude_ft() const;
        double get_heading() const;
        // Setter for brake input
        void set_input_brake(float value);

        // Getter for brake input (if needed)
        float get_input_brake() const;

        // Setter for flaps input
        void increase_flaps();
        void decrease_flaps();
        float get_flaps() const;

        // Propeller RPM from JSBSim
        double get_propeller_rpm() const;
    };

}

#endif