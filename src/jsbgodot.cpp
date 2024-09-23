#include "jsbgodot.h"
#include "models/FGFCS.h"
#include "models/FGAircraft.h"
#include "initialization/FGInitialCondition.h"
#include "models/FGPropulsion.h"

#include <godot_cpp/classes/input_event.hpp>        // Ensure InputEvent is fully included
#include <godot_cpp/classes/input_event_key.hpp>    // If you need specific event handling
#include <godot_cpp/core/memory.hpp>                // For Godot logging
#include <godot_cpp/classes/scene_tree.hpp>         // Include SceneTree header
#include <unistd.h>
#include <cmath>
#include <exception>

using namespace godot;

 const float EARTH_RADIUS = 6371000.0;  // Radius of Earth in meters

void JSBGodot::_bind_methods() {
  //  BIND_VIRTUAL_METHOD(JSBGodot, _process);
  //  BIND_VIRTUAL_METHOD(JSBGodot, _input);
 //   ClassDB::bind_method(D_METHOD("_input", "event"), &JSBGodot::_input);
    ClassDB::bind_method(D_METHOD("_process", "delta"), &JSBGodot::_process);
    ClassDB::bind_method(D_METHOD("set_input_pitch", "value"), &JSBGodot::set_input_pitch);
    ClassDB::bind_method(D_METHOD("get_input_pitch"), &JSBGodot::get_input_pitch);
    ClassDB::bind_method(D_METHOD("set_input_roll", "value"), &JSBGodot::set_input_roll);
    ClassDB::bind_method(D_METHOD("get_input_roll"), &JSBGodot::get_input_roll);
    ClassDB::bind_method(D_METHOD("set_input_rudder", "value"), &JSBGodot::set_input_rudder);
    ClassDB::bind_method(D_METHOD("get_input_rudder"), &JSBGodot::get_input_rudder);
    ClassDB::bind_method(D_METHOD("set_input_throttle", "value"), &JSBGodot::set_input_throttle);
    ClassDB::bind_method(D_METHOD("get_input_throttle"), &JSBGodot::get_input_throttle);

    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "input_pitch"), "set_input_pitch", "get_input_pitch");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "input_roll"), "set_input_roll", "get_input_roll");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "input_rudder"), "set_input_rudder", "get_input_rudder");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "input_throttle"), "set_input_throttle", "get_input_throttle");

}

JSBGodot::JSBGodot() {
    FDMExec = nullptr;
}

JSBGodot::~JSBGodot() {
    if (FDMExec) {
        delete FDMExec;
    }
}

void JSBGodot::set_input_pitch(float value) {
    input_pitch = value;
}

float JSBGodot::get_input_pitch() const {
    return input_pitch;
}

float JSBGodot::get_input_roll() const {
    return input_roll;
}

void JSBGodot::set_input_roll(float value) {
    input_roll = value;
}   

float JSBGodot::get_input_rudder() const {
    return input_rudder;
}   

void JSBGodot::set_input_rudder(float value) {
    input_rudder = value;
}

float JSBGodot::get_input_throttle() const {
    return input_throttle;
}

void JSBGodot::set_input_throttle(float value) {
    input_throttle = value;
}

void JSBGodot::_input(const Ref<InputEvent> event) {
    if (event->is_class("InputEventKey")) {
        Ref<InputEventKey> key_event = event;
        if (key_event->is_pressed()) {
            // Handle key press event
            int keycode = key_event->get_keycode();
             // Check if the 'O' or 'P' keys are pressed
            if (keycode == KEY_O) {
                // Decrease throttle
                set_input_throttle(input_throttle - 0.005f);  // Decrease by 0.1
                copy_outputs_from_JSBSim();
                printf("Throttle decreased to %f\n", input_throttle);
            } else if (keycode == KEY_P) {
                // Increase throttle
                set_input_throttle(input_throttle + 0.005f);  // Increase by 0.1
                copy_outputs_from_JSBSim();
                printf("Throttle increased to %f\n", input_throttle);
            } else if (keycode == KEY_W) {
                // Increase throttle
                set_input_pitch(input_pitch + 0.005f);  // Increase by 0.1
                copy_outputs_from_JSBSim();
                printf("Pitch increased to %f\n", input_pitch);
            } else if (keycode == KEY_S) {
                // Decrease throttle
                set_input_pitch(input_pitch - 0.005f);  // Decrease by 0.1
                copy_outputs_from_JSBSim();
                printf("Pitch decreased to %f\n", input_pitch);
            } else if (keycode == KEY_A) {
                // Decrease throttle
                set_input_roll(input_roll - 0.005f);  // Decrease by 0.1
                copy_outputs_from_JSBSim();
                printf("Roll decreased to %f\n", input_roll);
            } else if (keycode == KEY_D) {
                // Increase throttle
                set_input_roll(input_roll + 0.005f);  // Increase by 0.1
                copy_outputs_from_JSBSim();
                printf("Roll increased to %f\n", input_roll);
            }
             if (keycode == KEY_R) {
                // Decrease throttle
                set_process(true);
                printf("Process started\n");
             }
             if (keycode == KEY_E) {
                // Decrease throttle
                set_process(false);
                printf("Process ended\n");
             }
        }
    }
}

void JSBGodot::_process(double delta) {
    if (FDMExec) {
        copy_inputs_to_JSBSim();  // Apply inputs to JSBSim
        bool result = FDMExec->Run();  // Advance the simulation by one time step
        if (result) {
            double sim_time = FDMExec->GetSimTime();
            printf("Simulation time: %f\n", sim_time);
        } else {
            printf("FDMExec->Run() failed.\n");
        }
        copy_outputs_from_JSBSim();  // Update the position and rotation in Godot
    }
}

void JSBGodot::_physics_process(const real_t delta) {
    // Your physics logic
//    copy_inputs_to_JSBSim();
//    FDMExec->Run();
//    copy_outputs_from_JSBSim();
}

void JSBGodot::_ready() {
        printf("Initializing JSBGodot...\n");

        if (!FDMExec) {
            FDMExec = new JSBSim::FGFDMExec();
            if (FDMExec) {
                printf("FGFDMExec instance created successfully.\n");
            } else {
                printf("Failed to create FGFDMExec instance!\n");
                return;  // Prevent further execution if creation fails
            }
        }

        // Set JSBSim root directories with absolute paths for testing
        FDMExec->SetRootDir(SGPath("/Users/gerhardgubler/code/godotjbsim/jsbsim"));
        FDMExec->SetAircraftPath(SGPath("/Users/gerhardgubler/code/godotjbsim/jsbsim/aircraft"));
        FDMExec->SetEnginePath(SGPath("/Users/gerhardgubler/code/godotjbsim/jsbsim/engine"));
        FDMExec->SetSystemsPath(SGPath("/Users/gerhardgubler/code/godotjbsim/jsbsim/systems"));
        printf("JSBSim directories set.\n");

        // Load a known good aircraft model
        if (FDMExec->LoadModel(SGPath("aircraft"),
                               SGPath("engine"),
                               SGPath("systems"),
                               "c172x")) {
            printf("JSBSim model 'c172x' loaded successfully.\n");
        } else {
            printf("Failed to load JSBSim model 'c172x'.\n");
            return;  // Early exit if model loading fails
        }

        // Initialize the simulation
        initialise();
}

void JSBGodot::initialise() {
    std::shared_ptr<JSBSim::FGInitialCondition> ic = FDMExec->GetIC();

    // Set initial position
    ic->SetLatitudeDegIC(47.0);
    ic->SetLongitudeDegIC(-110.0);
    ic->SetAltitudeASLFtIC(45000.25 * 3.28084);  // Convert meters to feet

    // Set initial orientation
    ic->SetThetaDegIC(0.0);  // Pitch
    ic->SetPhiDegIC(0.0);    // Roll
    ic->SetPsiDegIC(0.0);    // Yaw

    // Set initial velocities (e.g., 200 m/s forward speed)
    ic->SetUBodyFpsIC(200.0 * 3.28084);  // Convert m/s to feet per second

    // Apply initial conditions
    FDMExec->RunIC();
}

void JSBGodot::copy_inputs_to_JSBSim() {
    if (!FDMExec) {
        printf("FDMExec is not initialized!\n");
        return;
    }

    // Retrieve the FCS and check for null before proceeding
    std::shared_ptr<JSBSim::FGFCS> FCS = FDMExec->GetFCS();
    if (!FCS) {
        printf("FCS object is null!\n");
        return;
    }

    // Apply input commands to the FCS
    FCS->SetDeCmd(input_pitch);
    FCS->SetDaCmd(input_roll);
    FCS->SetDrCmd(input_rudder);
    FCS->SetThrottleCmd(-1, input_throttle);
    printf("Inputs copied to JSBSim FCS. Throttle: %f\n", input_throttle);
}

Vector3 lat_lon_alt_to_cartesian(float latitude, float longitude, float altitude) {
    float lat_rad = Math::deg_to_rad(latitude);
    float lon_rad = Math::deg_to_rad(longitude);
    
    // Convert latitude, longitude, and altitude to Cartesian (ECEF)
    float x = (EARTH_RADIUS + altitude) * cos(lat_rad) * cos(lon_rad);
    float y = (EARTH_RADIUS + altitude) * cos(lat_rad) * sin(lon_rad);
    float z = (EARTH_RADIUS + altitude) * sin(lat_rad);
    
    return Vector3(x, y, z);
}

void JSBGodot::copy_outputs_from_JSBSim() {
    std::shared_ptr<JSBSim::FGPropagate> Propagate = FDMExec->GetPropagate();

    // Get aircraft altitude and position in geographic coordinates
    float altitude = Propagate->GetAltitudeASL();  // Altitude in meters
    float latitude = Propagate->GetLocation().GetLatitudeDeg();  // Latitude in degrees
    float longitude = Propagate->GetLocation().GetLongitudeDeg();  // Longitude in degrees

    // Print the geographic coordinates to ensure they're valid
    printf("Latitude: %f, Longitude: %f, Altitude: %f\n", latitude, longitude, altitude);

    // Convert latitude/longitude/altitude to Cartesian coordinates (ECEF)
    Vector3 position = lat_lon_alt_to_cartesian(latitude, longitude, altitude);

    // Use the initial position of the aircraft as the reference point
    static Vector3 reference_position = lat_lon_alt_to_cartesian(47.0, -110.0, 45000.25);  // Set your initial reference point here

    // Compute the local position relative to the reference point
    Vector3 local_position = position - reference_position;

    // Get Euler angles (in radians) for the aircraft's rotation
    float bank = Propagate->GetEuler(JSBSim::FGForce::ePhi);  // Roll angle in radians
    float pitch = Propagate->GetEuler(JSBSim::FGForce::eTht);  // Pitch angle in radians
    float heading = Propagate->GetEuler(JSBSim::FGForce::ePsi);  // Yaw angle in radians

    // Cast the parent to Node3D
    Node3D *parent_node = Object::cast_to<Node3D>(get_parent());
    if (parent_node) {
        // Set the position of the parent (the AC Node3D)
        parent_node->set_position(local_position);

        // Set the rotation of the parent (the AC Node3D)
        Vector3 newRot = Vector3(pitch, heading, bank);
        parent_node->set_rotation(newRot);

        printf("Updated parent (AC) position to x: %f, y: %f, z: %f\n", local_position.x, local_position.y, local_position.z);
    } else {
        printf("Parent is not a Node3D!\n");
    }
}