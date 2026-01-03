# JSBSim GDExtension for Godot 4

This repository is a reusable Godot 4 GDExtension that integrates the
[JSBSim](https://github.com/JSBSim-Team/jsbsim) flight dynamics model.

It also contains a small demo Godot project that exercises the extension.

## What’s in here

- GDExtension C++ source in `src/` (class `JSBGodot`)
- A minimal demo project in `gojb/` (open in Godot to try it)
- Submodules:
  - `jsbsim/` (flight dynamics)
  - `godot-cpp/` (Godot C++ bindings)

This repo intentionally does not depend on Terrain3D or large external scenery datasets.

## Build

### Prerequisites

- Godot 4.6+ (4.3+ is supported by the `.gdextension` config)
- Python 3 + SCons
- A C++ toolchain (Clang/GCC/MSVC)

### Steps

1. Clone with submodules:
   ```bash
   git clone --recursive git@gitlab.md80.ch:gery/godotjsbsim.git
   cd godotjsbsim
   ```

2. Build JSBSim (first time only):
   ```bash
   cd jsbsim
   mkdir -p build && cd build
   cmake ..
   cmake --build . -j
   cd ../..
   ```

3. Build the extension:
   ```bash
   # macOS (Apple Silicon)
   scons arch=arm64
   
   # macOS (Apple x86)
   scons arch=x64_64

   # Linux
   scons 
   ```

4. Run the demo:
   ```bash
   cd gojb
   godot --editor
   ```

## Using the plugin in your own project

The demo project contains the GDExtension config at `gojb/bin/gojb.gdextension`.
For your own project, you typically copy (or vendor) the extension binary + `.gdextension`
into your project and instantiate the `JSBGodot` node.

Note: the demo currently resolves JSBSim data from `res://../jsbsim` (repo layout).
If you package/export a project, you’ll likely want to ship JSBSim data in a dedicated
location (e.g. `user://` or an unpacked folder) and point JSBSim at that path.

## License

This project combines multiple open-source components:
- JSBSim: LGPL 2.1
- Godot Engine: MIT
- godot-cpp: MIT


## Aircraft model

* Title:	   Cessna172
* Source:	https://sketchfab.com/3d-models/cessna172-d1b15841c29c43d0862667300bad55a4
* Author:	KOG_THORNS (https://sketchfab.com/ioai25312)

