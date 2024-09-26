#include "jsbgodot.h"
#include "models/FGFCS.h"
#include "models/FGAircraft.h"
#include "initialization/FGInitialCondition.h"
#include "models/FGPropulsion.h"
#include "models/propulsion/FGThruster.h"

#include <godot_cpp/classes/input_event.hpp>     // Ensure InputEvent is fully included
#include <godot_cpp/classes/input_event_key.hpp> // If you need specific event handling
#include <godot_cpp/core/memory.hpp>             // For Godot logging
#include <godot_cpp/classes/scene_tree.hpp>      // Include SceneTree header
#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/classes/scene_tree.hpp>
#include <unistd.h>
#include <cmath>
#include <exception>

using namespace godot;

const float EARTH_RADIUS = 6371000.0; // Radius of Earth in meters

void JSBGodot::_bind_methods()
{
    ClassDB::bind_method(D_METHOD("_process", "delta"), &JSBGodot::_process);
    ClassDB::bind_method(D_METHOD("set_input_pitch", "value"), &JSBGodot::set_input_pitch);
    ClassDB::bind_method(D_METHOD("get_input_pitch"), &JSBGodot::get_input_pitch);
    ClassDB::bind_method(D_METHOD("set_input_roll", "value"), &JSBGodot::set_input_roll);
    ClassDB::bind_method(D_METHOD("get_input_roll"), &JSBGodot::get_input_roll);
    ClassDB::bind_method(D_METHOD("set_input_rudder", "value"), &JSBGodot::set_input_rudder);
    ClassDB::bind_method(D_METHOD("get_input_rudder"), &JSBGodot::get_input_rudder);
    ClassDB::bind_method(D_METHOD("set_input_throttle", "value"), &JSBGodot::set_input_throttle);
    ClassDB::bind_method(D_METHOD("get_input_throttle"), &JSBGodot::get_input_throttle);
    ClassDB::bind_method(D_METHOD("get_airspeed_knots"), &JSBGodot::get_airspeed_knots);
    ClassDB::bind_method(D_METHOD("get_vertical_speed_fpm"), &JSBGodot::get_vertical_speed_fpm);
    ClassDB::bind_method(D_METHOD("get_altitude_ft"), &JSBGodot::get_altitude_ft);
    ClassDB::bind_method(D_METHOD("get_heading"), &JSBGodot::get_heading);
    ClassDB::bind_method(D_METHOD("set_input_brake", "value"), &JSBGodot::set_input_brake);
    ClassDB::bind_method(D_METHOD("get_input_brake"), &JSBGodot::get_input_brake);

    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "input_pitch"), "set_input_pitch", "get_input_pitch");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "input_roll"), "set_input_roll", "get_input_roll");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "input_rudder"), "set_input_rudder", "get_input_rudder");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "input_throttle"), "set_input_throttle", "get_input_throttle");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "airspeed_knots", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_DEFAULT), "", "get_airspeed_knots");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "vertical_speed_fpm", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_DEFAULT), "", "get_vertical_speed_fpm");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "altitude_ft", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_DEFAULT), "", "get_altitude_ft");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "heading", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_DEFAULT), "", "get_heading");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "input_brake"), "set_input_brake", "get_input_brake");
}

JSBGodot::JSBGodot()
{
    FDMExec = nullptr;
}

JSBGodot::~JSBGodot()
{
    if (FDMExec)
    {
        delete FDMExec;
    }
}

void JSBGodot::set_input_pitch(float value)
{
    if (value < -1.0f)
    {
        input_pitch = -1.0f;
    }
    else if (value > 1.0f)
    {
        input_pitch = 1.0f;
    }
    else
    {
        input_pitch = value;
    }
}

float JSBGodot::get_input_pitch() const
{
    return input_pitch;
}

float JSBGodot::get_input_roll() const
{
    return input_roll;
}

void JSBGodot::set_input_roll(float value)
{
    if (value < -1.0f)
    {
        input_roll = -1.0f;
    }
    else if (value > 1.0f)
    {
        input_roll = 1.0f;
    }
    else
    {
       input_roll = value;
    }
}

float JSBGodot::get_input_rudder() const
{
    return input_rudder;
}

void JSBGodot::set_input_rudder(float value)
{
    if (value < -1.0f)
    {
        input_rudder = -1.0f;
    }
    else if (value > 1.0f)
    {
        input_rudder = 1.0f;
    }
    else
    {
        input_rudder = value;
    }
}

float JSBGodot::get_input_throttle() const
{
    return input_throttle;
}

void JSBGodot::set_input_throttle(float value)
{
    if (value < 0.0f)
    {
        input_throttle = 0.0f;
    }
    else if (value > 1.0f)
    {
        input_throttle = 1.0f;
    }
    else
    {
        input_throttle = value;
    }
}

void JSBGodot::set_input_aileron(float value) {
    //input_aileron = CLAMP(value, -1.0f, 1.0f);
    if (value < -1.0f)
    {
        input_aileron = -1.0f;
    }
    else if (value > 1.0f)
    {
        input_aileron = 1.0f;
    }
    else
    {
     input_aileron = value;
    }
}

void JSBGodot::set_input_elevator(float value) {
   // input_elevator = CLAMP(value, -1.0f, 1.0f);
   if (value < -1.0f)
    {
        input_elevator = -1.0f;
    }
    else if (value > 1.0f)
    {
        input_elevator = 1.0f;
    }
    else
    {
        input_elevator = value;
    }
}

void JSBGodot::set_input_brake(float value) {
    input_brake = CLAMP(value, 0.0f, 1.0f);
}

float JSBGodot::get_input_brake() const {
    return input_brake;
}

double JSBGodot::get_airspeed_knots() const {
    return airspeed_knots;
}

double JSBGodot::get_vertical_speed_fpm() const {
    return vertical_speed_fpm;
}

double JSBGodot::get_altitude_ft() const {
    return altitude_ft;
}

double JSBGodot::get_heading() const {
    return heading_deg;
}

void JSBGodot::_input(const Ref<InputEvent> event)
{
    if (event->is_class("InputEventKey"))
    {
        Ref<InputEventKey> key_event = event;
        if (key_event->is_pressed())
        {
            // Handle key press event
            int keycode = key_event->get_keycode();
            // Check if the 'O' or 'P' keys are pressed
            if (keycode == KEY_O)
            {
                // Decrease throttle
                set_input_throttle(input_throttle - 0.05f); // Decrease by 0.1
                copy_outputs_from_JSBSim();
                printf("Throttle decreased to %f\n", input_throttle);
            }
            else if (keycode == KEY_P)
            {
                // Increase throttle
                set_input_throttle(input_throttle + 0.05f); // Increase by 0.1
                copy_outputs_from_JSBSim();
                printf("Throttle increased to %f\n", input_throttle);
            }
            else if (keycode == KEY_W)
            {
                // Increase Elevator
                set_input_elevator(input_elevator + 0.05f); // Increase by 0.1
                copy_outputs_from_JSBSim();
                printf("Pitch increased to %f\n", input_pitch);
            }
            else if (keycode == KEY_S)
            {
                // Decrease Elevator
                set_input_elevator(input_elevator - 0.05f); // Decrease by 0.1
                copy_outputs_from_JSBSim();
                printf("Pitch decreased to %f\n", input_pitch);
            }
            else if (keycode == KEY_D)
            {
                // Increase Aileron
                set_input_aileron(input_aileron - 0.05f); // Decrease by 0.1
                copy_outputs_from_JSBSim();
                printf("Roll decreased to %f\n", input_roll);
            }
            else if (keycode == KEY_A)
            {
                // Decrease Aileron
                set_input_aileron(input_aileron + 0.05f); // Increase by 0.1
                copy_outputs_from_JSBSim();
                printf("Roll increased to %f\n", input_roll);
            }
            else if (keycode == KEY_X)
            {
                // Right Rudder
                set_input_rudder(input_rudder + 0.05f); // Increase by 0.1
            }
            else if (keycode == KEY_Y)
            {
                // Left Rudder
                set_input_rudder(input_rudder - 0.05f); // Decrease by 0.1
            }
            if (keycode == KEY_R)
            {
                // Start process
                set_process(true);
                printf("Process started\n");
            }
            if (keycode == KEY_E)
            {
                // Stop process
                set_process(false);
                printf("Process ended\n");
            }
            if (keycode == KEY_PERIOD)
            {
                set_input_brake(1.0f);
                printf("Brakes applied\n");
            }
            else if (keycode == KEY_COMMA)
            {
                set_input_brake(0.0f);
                printf("Brakes released\n");
            }
        }
    }
}

void JSBGodot::_process(double delta)
{
    //    if (FDMExec) {
    //        copy_inputs_to_JSBSim();  // Apply inputs to JSBSim
    //        bool result = FDMExec->Run();  // Advance the simulation by one time step
    //        if (result) {
    //            double sim_time = FDMExec->GetSimTime();
    //            printf("Simulation time: %f\n", sim_time);
    //        } else {
    //            printf("FDMExec->Run() failed.\n");
    //        }
    //        copy_outputs_from_JSBSim();  // Update the position and rotation in Godot
    //    }
}

void JSBGodot::_physics_process(const real_t delta)
{
    //    Your physics logic
    if (!Engine::get_singleton()->is_editor_hint())
    {
        copy_inputs_to_JSBSim();
        FDMExec->Run();
        copy_outputs_from_JSBSim();
    }
}

void JSBGodot::_ready()
{
    printf("Initializing JSBGodot...\n");

    if (!FDMExec)
    {
        FDMExec = new JSBSim::FGFDMExec();
        if (FDMExec)
        {
            printf("FGFDMExec instance created successfully.\n");
        }
        else
        {
            printf("Failed to create FGFDMExec instance!\n");
            return; // Prevent further execution if creation fails
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
                           "c172p"))
    {
        printf("JSBSim model 'c172p' loaded successfully.\n");
    }
    else
    {
        printf("Failed to load JSBSim model 'c172p'.\n");
        return; // Early exit if model loading fails
    }

    // Initialize the simulation
    initialise();
}

void JSBGodot::initialise()
{
    std::shared_ptr<JSBSim::FGInitialCondition> ic = FDMExec->GetIC();

    // Set initial position
    ic->SetLatitudeDegIC(0.0);
    ic->SetLongitudeDegIC(0.0);
    ic->SetAltitudeASLFtIC(1 * 3.28084); // Convert meters to feet

    // Set initial orientation
    ic->SetThetaDegIC(0.0); // Pitch
    ic->SetPhiDegIC(0.0);   // Roll
    ic->SetPsiDegIC(0.0);   // Yaw

    // Set initial velocities (e.g., 200 m/s forward speed)
    ic->SetUBodyFpsIC(0.0 * 3.28084); // Convert m/s to feet per second
    // Start the engine
    FDMExec->SetPropertyValue("propulsion/engine[0]/set-running", 1);
    // Release brakes
    FDMExec->SetPropertyValue("fcs/brake-cmd-norm", 0.0);
    // Ensure parking brake is released
    FDMExec->SetPropertyValue("fcs/parking-brake-cmd-norm", 0.0);
    // Set mixture to full rich
    FDMExec->SetPropertyValue("fcs/mixture-cmd-norm", 1.0);
    // Set magnetos to both
    FDMExec->SetPropertyValue("propulsion/magneto_cmd", 3);  // 3 = Both
    // Start the engine
    FDMExec->SetPropertyValue("propulsion/starter_cmd", 1.0);
    // Ensure the pitot-static system is enabled (example property)
    FDMExec->SetPropertyValue("sensors/pitot_static/enable", 1);

    // Apply initial conditions
    FDMExec->RunIC();
}

void JSBGodot::copy_inputs_to_JSBSim()
{
    if (!FDMExec)
    {
        printf("FDMExec is not initialized!\n");
        return;
    }

    // Retrieve the FCS and check for null before proceeding
    std::shared_ptr<JSBSim::FGFCS> FCS = FDMExec->GetFCS();
    if (!FCS)
    {
        printf("FCS object is null!\n");
        return;
    }

    // Apply input commands to the FCS
//    FCS->SetDeCmd(input_pitch);
//    FCS->SetDaCmd(input_roll);
//    FCS->SetDrCmd(input_rudder);
//    FCS->SetThrottleCmd(-1, input_throttle);
//    printf("Inputs copied to JSBSim FCS. Throttle: %f\n", input_throttle);

        // Set control surface deflections via the property tree
    FDMExec->SetPropertyValue("fcs/aileron-cmd-norm", input_aileron);
    FDMExec->SetPropertyValue("fcs/elevator-cmd-norm", input_elevator);
    FDMExec->SetPropertyValue("fcs/rudder-cmd-norm", input_rudder);
    FDMExec->SetPropertyValue("fcs/throttle-cmd-norm", input_throttle); 

        // Set the brake command
    FDMExec->SetPropertyValue("fcs/left-brake-cmd-norm", input_brake);
    FDMExec->SetPropertyValue("fcs/right-brake-cmd-norm", input_brake);

    printf("Control inputs applied: Aileron=%f, Elevator=%f, Rudder=%f, Throttle=%f\n",
           input_aileron, input_elevator, input_rudder, input_throttle);
}

Vector3 lat_lon_alt_to_cartesian(float latitude, float longitude, float altitude)
{
    float lat_rad = Math::deg_to_rad(latitude);
    float lon_rad = Math::deg_to_rad(longitude);

    // Convert latitude, longitude, and altitude to Cartesian (ECEF)
    float x = (EARTH_RADIUS + altitude) * cos(lat_rad) * cos(lon_rad);
    float y = (EARTH_RADIUS + altitude) * cos(lat_rad) * sin(lon_rad);
    float z = (EARTH_RADIUS + altitude) * sin(lat_rad);

    return Vector3(x, y, z);
}

Vector3 lat_lon_alt_to_local(float latitude, float longitude, float altitude)
{
    static bool reference_set = false;
    static double ref_lat_rad, ref_lon_rad, ref_alt;

    double lat_rad = Math::deg_to_rad(latitude);
    double lon_rad = Math::deg_to_rad(longitude);

    if (!reference_set) {
        ref_lat_rad = lat_rad;
        ref_lon_rad = lon_rad;
        ref_alt = altitude;
        reference_set = true;
    }

    double d_lat = lat_rad - ref_lat_rad;
    double d_lon = lon_rad - ref_lon_rad;
    double d_alt = altitude - ref_alt;

    const double R = 6378137.0; // Earth's radius in meters

    double north = d_lat * R;
    double east = d_lon * R * cos(ref_lat_rad);
    double up = d_alt;

    // Map to Godot's coordinate system (X-East, Y-Up, Z-North)
    return Vector3(east, up, north);
}

void JSBGodot::copy_outputs_from_JSBSim()
{
   // Ensure FDMExec and Propagate are valid
    if (!FDMExec) {
        printf("FDMExec is null!\n");
        return;
    }
    auto Propagate = FDMExec->GetPropagate();
    if (!Propagate) {
        printf("Propagate is null!\n");
        return;
    }

    // Get aircraft position in geographic coordinates
    double latitude = Propagate->GetLatitudeDeg();
    double longitude = Propagate->GetLongitudeDeg();
    altitude_ft = Propagate->GetAltitudeASL();

    // Convert altitude from feet to meters
    double altitude_m = altitude_ft * 0.3048;

    // Print the geographic coordinates
    // printf("Latitude: %f, Longitude: %f, Altitude: %f ft\n", latitude, longitude, altitude_ft);

    // Get local position in meters
    Vector3 local_position = lat_lon_alt_to_local(latitude, longitude, altitude_m);

    // Get Euler angles (in radians) from JSBSim
    double bank = Propagate->GetEuler(1);    // Roll
    double pitch = Propagate->GetEuler(2);   // Pitch
    double heading = Propagate->GetEuler(3); // Yaw

    // Map Euler angles to Godot's coordinate system
    Vector3 newRot = Vector3(-pitch, heading, -bank); // Adjust signs as needed

    // Get the parent node
    Node3D *parent_node = Object::cast_to<Node3D>(get_parent());
    if (!parent_node) {
        printf("Parent node is null!\n");
        return;
    }

    // Update the position and rotation of the parent node
    parent_node->set_position(local_position);
    parent_node->set_rotation(newRot);

    printf("Updated parent (AC) position to x: %f, y: %f, z: %f\n",
           local_position.x, local_position.y, local_position.z);

        // Access airspeed from the property tree
    double tas_knots = FDMExec->GetPropertyValue("aero/qbar-psf");
    double ias_knots = FDMExec->GetPropertyValue("velocities/vc-kts");
    
    // Access vertical speed (v-down-fps is positive downwards)
    double vertical_speed_fps = -FDMExec->GetPropertyValue("velocities/v-down-fps");
    vertical_speed_fpm = vertical_speed_fps * 60.0;

    double engine_thrust = FDMExec->GetPropulsion()->GetEngine(0)->GetThruster()->GetThrust();

    printf("Engine Thrust: %f N\n", engine_thrust);
    printf("roll: %f, pitch: %f, yaw: %f\n", bank, pitch, heading);
    
    airspeed_knots = FDMExec->GetPropertyValue("velocities/vc-kts");
        
    // Get the heading in degrees from the property tree
    heading_deg = FDMExec->GetPropertyValue("attitude/psi-deg");
    heading_deg = fmod(360.0 - heading_deg, 360.0);

    // Convert heading to radians if needed
    double heading_rad = Math::deg_to_rad(heading_deg);

    printf("heading %f\n", heading_deg);
    printf("heading_rad %f\n", heading_rad);

}