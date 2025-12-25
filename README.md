# JSBSim Flight Dynamics for Godot 4

A GDExtension that integrates [JSBSim](https://github.com/JSBSim-Team/jsbsim) flight dynamics model into Godot 4.6+.

## Overview

This project provides a flight simulation framework using JSBSim as the physics engine and Godot as the 3D rendering engine. It allows you to fly realistic aircraft with accurate flight dynamics in a Godot game environment.

Originally based on [chunky/jsbgodot](https://github.com/chunky/jsbgodot/tree/master/demo) for Godot 3, this version has been updated for Godot 4.6.

## Features

- **Realistic Flight Dynamics**: Full 6-DOF flight simulation powered by JSBSim
- **Gamepad Support**: Full joystick/gamepad control with configurable axes
- **Control Surface Animation**: Animated ailerons, elevators, rudder, flaps, and propeller
- **Multiple Camera Views**:
  - Cockpit camera with head look (hat switch / D-pad / arrow keys)
  - External orbit camera with zoom (shoulder buttons / Page Up/Down)
- **C172 Skyhawk**: Includes a Cessna 172 aircraft model with JSBSim flight model

## Controls

### Flight Controls
| Control | Gamepad | Keyboard |
|---------|---------|----------|
| Pitch (Elevator) | Left Stick Y | W/S |
| Roll (Aileron) | Left Stick X | A/D |
| Rudder | Right Stick X | Q/E |
| Throttle | Right Stick Y | +/- |
| Flaps Up | D-pad Up | F |
| Flaps Down | D-pad Down | V |
| Brakes | Button | B |

### Camera Controls
| Control | Gamepad | Keyboard |
|---------|---------|----------|
| Switch Camera | Select/Back | C |
| Head Look (Cockpit) | Hat Switch / D-pad | Arrow Keys |
| Orbit Camera (External) | Hat Switch / D-pad | Arrow Keys |
| Zoom In | L1 (Left Shoulder) | Page Up |
| Zoom Out | R1 (Right Shoulder) | Page Down |

## Building

### Prerequisites

- Godot 4.6+ (dev builds work)
- Python 3.x with SCons
- C++ compiler (Clang on macOS, GCC on Linux, MSVC on Windows)
- JSBSim library (included as submodule)

### Build Steps

1. Clone the repository with submodules:
   ```bash
   git clone --recursive https://github.com/yourusername/godotjbsim.git
   cd godotjbsim
   ```

2. Build JSBSim (if not already built):
   ```bash
   cd jsbsim
   mkdir build && cd build
   cmake ..
   make -j8
   cd ../..
   ```

3. Build the GDExtension:
   ```bash
   # macOS ARM64
   scons arch=arm64 -j8
   
   # macOS x86_64
   scons arch=x86_64 -j8
   
   # Linux
   scons -j8
   ```

4. Copy the built library to the project:
   ```bash
   cp libgojb.macos.template_debug.arm64.dylib gojb/bin/
   ```

5. Open the project in Godot:
   ```bash
   cd gojb
   godot --editor
   ```

## Project Structure

```
godotjbsim/
├── src/                    # GDExtension C++ source
│   ├── jsbgodot.cpp       # JSBSim integration
│   ├── jsbgodot.h
│   └── register_types.cpp
├── gojb/                   # Godot project
│   ├── bin/               # GDExtension binaries
│   ├── Assets/            # 3D models and textures
│   ├── node_3d.gd         # Main scene script
│   ├── node_3d.tscn       # Main scene
│   └── project.godot
├── jsbsim/                 # JSBSim submodule
├── godot-cpp/              # Godot C++ bindings submodule
└── SConstruct              # Build configuration
```

## Control Surface Animation

The aircraft model supports animated control surfaces:
- **Propeller**: Spins based on throttle (600-2700 RPM) with blur disc at high RPM
- **Ailerons**: ±20° deflection, opposite movement for roll
- **Elevators**: ±25° deflection
- **Rudder**: ±16° deflection
- **Flaps**: 0°, 10°, 20°, 30°, 40° (C172 standard positions)

### Propeller Animation

The propeller uses a visibility-swapping technique for realistic appearance:
- **Below 900 RPM**: Real propeller blades visible, spinning
- **900-1200 RPM**: Transition zone with both blades and blur disc visible
- **Above 1200 RPM**: Blur disc only (semi-transparent circular disc)

This technique is commonly used in flight simulators to avoid the "wagon wheel" 
effect of spinning propellers and provide a more realistic visual representation
at high RPM.

To animate control surfaces in your own aircraft model:
1. Separate control surfaces as individual mesh objects in Blender
2. Set the origin/pivot point at the hinge line
3. Align the local axes so rotation works correctly
4. Export as glTF and import into Godot

## Aircraft Models

### Included
- **C172 Skyhawk**: Cessna 172 with full control surface animation

### Adding New Aircraft
1. Add JSBSim aircraft XML files to `jsbsim/aircraft/`
2. Import 3D model into `gojb/Assets/`
3. Update node paths in `node_3d.gd` for control surface animation
4. Modify `jsbgodot.cpp` if needed for aircraft-specific features

## Known Issues

- Aircraft must be positioned above ground level at start to avoid physics issues
- Some JSBSim aircraft models may not have fully functional flaps
- Debug output is verbose (can be reduced by commenting out printf statements)

## License

This project combines multiple open-source components:
- JSBSim: LGPL 2.1
- Godot Engine: MIT
- godot-cpp: MIT
- Aircraft models: Various (see individual model licenses)

## Acknowledgments

- [JSBSim Team](https://github.com/JSBSim-Team/jsbsim) for the flight dynamics engine
- [chunky/jsbgodot](https://github.com/chunky/jsbgodot) for the original Godot 3 integration
- Sketchfab artists for aircraft models
Model Information:
* title:	Cessna172
* source:	https://sketchfab.com/3d-models/cessna172-d1b15841c29c43d0862667300bad55a4
* author:	KOG_THORNS (https://sketchfab.com/ioai25312)