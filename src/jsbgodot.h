#ifndef JSBGODOT_H
#define JSBGODOT_H

#include "FGFDMExec.h"

#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/classes/input_event.hpp>  // Ensure InputEvent is fully included

namespace godot {

class JSBGodot : public Node3D {
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
    void _input(const Ref<InputEvent> event);  // Using Ref<InputEvent> requires full inclusion
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
    void set_input_aileron(float value);

};

}

#endif