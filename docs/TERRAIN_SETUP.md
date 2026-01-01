# Terrain Setup and JSBSim Collision Detection

This document describes how to import terrain data and how the collision detection between Godot's Terrain3D and JSBSim works.

## Overview

The flight simulator uses:
- **Godot Terrain3D** for visual terrain rendering and collision queries
- **JSBSim** for flight dynamics and ground reaction physics

These two systems need to share terrain elevation data so the aircraft can properly interact with the ground.

## Terrain Import Procedure

### 1. Terrain Data Source

We use Swiss high-resolution terrain data (swissALTI3D) in R16 format:
- **File**: `zurich_airport_hires.r16`
- **Resolution**: 0.5m per pixel
- **Format**: 16-bit unsigned integer heightmap

### 2. Import Settings in Terrain3D

1. Open Godot and select the **Terrain3D** node
2. Go to **Terrain3D → Tools → Import**
3. Configure the import settings:

| Setting | Value | Notes |
|---------|-------|-------|
| **File** | `zurich_airport_hires.r16` | Your heightmap file |
| **Height Offset** | `0` | Import with no offset for simplest setup |
| **Height Scale** | `1.0` | Use real-world heights |
| **Import Position** | `(0, 0)` | Center of terrain |

4. Click **Import**

### 3. Understanding Height Offset

The **Height Offset** during import shifts all terrain vertices vertically:

| Import Offset | Godot Terrain Y | JSBSim Offset Needed | Use Case |
|---------------|-----------------|----------------------|----------|
| **0** | Real ASL elevation | `0` | Simplest - recommended |
| **-480** | Near Y=0 | `480` | If Terrain3D has world bounds issues |

**Current Setup**: Import offset = 0, JSBSim offset = 0

This means Godot terrain Y coordinates directly equal real-world ASL (Above Sea Level) elevation in meters.

## Collision Detection Architecture

### How It Works

```
┌─────────────────────────────────────────────────────────────────┐
│                         Per Frame                                │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  1. Get aircraft position (AC.global_position)                  │
│                           │                                      │
│                           ▼                                      │
│  2. Query Terrain3D height at that XZ position                  │
│     terrain_height = terrain_data.get_height(ac_position)       │
│                           │                                      │
│                           ▼                                      │
│  3. Convert to JSBSim ASL elevation                             │
│     jsbsim_elevation = terrain_height + jsbsim_elevation_offset │
│                           │                                      │
│                           ▼                                      │
│  4. Update JSBSim terrain property                              │
│     set_terrain_elevation(jsbsim_elevation)                     │
│                           │                                      │
│                           ▼                                      │
│  5. JSBSim runs physics with correct ground height              │
│     - Ground reaction forces                                     │
│     - Gear compression                                           │
│     - Altimeter reading                                          │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

### Key Components

#### GDScript (node_3d.gd)

```gdscript
# Offset to convert Godot Y to JSBSim ASL elevation
var jsbsim_elevation_offset: float = 0.0

func _update_terrain_elevation() -> void:
    # Get terrain height at aircraft position
    var terrain_height_godot = terrain_data.get_height(ac_position)
    
    # Convert to JSBSim ASL
    var terrain_elevation_jsbsim = terrain_height_godot + jsbsim_elevation_offset
    
    # Update JSBSim
    jsb_node.set_terrain_elevation(terrain_elevation_jsbsim)
```

#### C++ (jsbgodot.cpp)

```cpp
void JSBGodot::set_terrain_elevation(double elevation_m) {
    double elevation_ft = elevation_m * 3.28084;
    FDMExec->SetPropertyValue("position/terrain-elevation-asl-ft", elevation_ft);
}
```

### Coordinate Transformation

JSBSim outputs aircraft position relative to its initial reference point. To display the aircraft at the correct Godot world position:

```cpp
// In copy_outputs_from_JSBSim()
Vector3 local_position = lat_lon_alt_to_local(latitude, longitude, altitude_m);
local_position.y += godot_terrain_y_offset;  // Transform to Godot world
parent_node->set_position(local_position);
```

The `godot_terrain_y_offset` is set at initialization to the terrain height at the starting position.

## Configuration Variables

### In node_3d.gd

| Variable | Default | Purpose |
|----------|---------|---------|
| `jsbsim_elevation_offset` | `0.0` | Added to Godot Y to get JSBSim ASL |
| `WHEEL_GROUND_CLEARANCE` | `0.6` | Meters to lift aircraft to prevent wheel clipping |
| `TERRAIN_INIT_DELAY_FRAMES` | `60` | Frames to wait before per-frame terrain updates |

### In JSBGodot C++

| Variable | Purpose |
|----------|---------|
| `godot_terrain_y_offset` | Shifts JSBSim output Y to Godot world coordinates |

## Troubleshooting

### Aircraft falls through terrain
- Check that `_update_terrain_elevation()` is being called (not disabled)
- Verify Terrain3D collision mode is set to "Dynamic" or "Game"
- Check terrain data is loaded (`region_count > 0`)

### Altimeter shows wrong altitude
- Verify `jsbsim_elevation_offset` matches your import offset
- With import offset 0: JSBSim offset should be 0
- With import offset -480: JSBSim offset should be 480

### Wheels clip through ground
- Increase `WHEEL_GROUND_CLEARANCE` (default 0.6m)

### Terrain too high/low in Godot world
- Adjust import height offset in Terrain3D import settings
- Remember to update `jsbsim_elevation_offset` to match

## File Locations

- **Terrain data**: `gojb/Terra3DData/` (Terrain3D .res files)
- **Heightmap source**: `gojb/zurich_airport_hires.r16`
- **Main script**: `gojb/node_3d.gd`
- **JSBSim binding**: `src/jsbgodot.cpp`, `src/jsbgodot.h`
