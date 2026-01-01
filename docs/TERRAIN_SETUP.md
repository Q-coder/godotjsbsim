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
- **Elevation**: Real ASL (Above Sea Level) values (~424m at Zurich Airport)

### 2. Import Settings in Terrain3D

1. Open Godot and select the **Terrain3D** node
2. Go to **Terrain3D → Tools → Import**
3. Configure the import settings:

| Setting | Value | Notes |
|---------|-------|-------|
| **File** | `zurich_airport_hires.r16` | Your heightmap file |
| **Height Offset** | `0` | Import with no offset - Godot Y = real ASL |
| **Height Scale** | `1.0` | Use real-world heights |
| **Import Position** | `(0, 0)` | Center of terrain |

4. Click **Import**

### 3. Understanding the Coordinate System

With **Height Offset = 0**:
- Godot terrain Y coordinates = real-world ASL elevation in meters
- JSBSim altitude (meters) = Godot Y position directly
- No complex offset calculations needed

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
│  3. Update JSBSim terrain elevation (direct, no offset)         │
│     set_terrain_elevation(terrain_height)                       │
│                           │                                      │
│                           ▼                                      │
│  4. JSBSim runs physics with correct ground height              │
│     - Ground reaction forces                                     │
│     - Gear compression                                           │
│     - Altimeter reading                                          │
│                                                                  │
│  5. JSBSim outputs altitude → Godot Y position                  │
│     Godot Y = altitude_m + WHEEL_GROUND_CLEARANCE               │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

### Key Components

#### GDScript (node_3d.gd)

```gdscript
# Wheel clearance to prevent visual clipping
const WHEEL_GROUND_CLEARANCE: float = 0.5  # meters

func _do_deferred_terrain_init() -> void:
    # Set the wheel clearance offset for visual positioning
    jsb_node.set_godot_terrain_y_offset(WHEEL_GROUND_CLEARANCE)

func _update_terrain_elevation() -> void:
    # Get terrain height at aircraft position (already in ASL meters)
    var terrain_height = terrain_data.get_height(ac_position)
    
    # Send directly to JSBSim - no offset needed
    jsb_node.set_terrain_elevation(terrain_height)
```

#### C++ (jsbgodot.cpp)

```cpp
void JSBGodot::set_terrain_elevation(double elevation_m) {
    double elevation_ft = elevation_m * 3.28084;
    FDMExec->SetPropertyValue("position/terrain-elevation-asl-ft", elevation_ft);
}

// In copy_outputs_from_JSBSim()
// Godot Y = JSBSim altitude (meters) + wheel clearance offset
local_position.y = altitude_m + godot_terrain_y_offset;
```

### Coordinate Systems

#### Swiss LV95 to Godot World

The terrain is centered at Swiss LV95 coordinates, converted to Godot:
- **Terrain Center E**: 2686872.0 (Easting at Godot X=0)
- **Terrain Center N**: 1257719.0 (Northing at Godot Z=0)

Conversion formulas:
```
Godot X = TERRAIN_CENTER_E - Swiss_E  (+X = West)
Godot Z = Swiss_N - TERRAIN_CENTER_N  (+Z = North)
```

#### Godot to WGS84 (Lat/Lon)

The HUD displays real-world coordinates by:
1. Converting Godot position back to Swiss LV95
2. Using swisstopo approximate formulas to convert LV95 → WGS84

```gdscript
func lv95_to_wgs84(easting: float, northing: float) -> Vector2:
    # Returns Vector2(latitude, longitude) in degrees
```

## Configuration Variables

### In node_3d.gd

| Variable | Value | Purpose |
|----------|-------|---------|
| `WHEEL_GROUND_CLEARANCE` | `0.5` | Meters to lift visual model to prevent wheel clipping |
| `TERRAIN_CENTER_E` | `2686872.0` | Swiss Easting at Godot X=0 |
| `TERRAIN_CENTER_N` | `1257719.0` | Swiss Northing at Godot Z=0 |

### In JSBGodot C++

| Variable | Purpose |
|----------|---------|
| `godot_terrain_y_offset` | Small offset (e.g., 0.5m) to lift visual model above ground |

## HUD Display

The simulator displays:
- **Godot coordinates**: X, Y, Z in world space
- **Lat/Lon**: Real-world WGS84 coordinates calculated from position
- **Altitude**: JSBSim altitude in feet ASL

## Troubleshooting

### Aircraft falls through terrain
- Check that `_update_terrain_elevation()` is being called
- Verify Terrain3D collision mode is set to "Full"
- Check terrain data is loaded (`region_count > 0`)

### Wheels clip through ground
- Increase `WHEEL_GROUND_CLEARANCE` in node_3d.gd (default 0.5m)

### Lat/Lon display incorrect
- Verify `TERRAIN_CENTER_E` and `TERRAIN_CENTER_N` match your terrain import
- These should be the Swiss LV95 coordinates at Godot position (0, 0)

### Buildings floating or underground
- Adjust `building_offset` on the BuildingLoader node
- Buildings use the same coordinate system as terrain

## File Locations

- **Terrain data**: `gojb/Terra3DData/` (Terrain3D .res files)
- **Heightmap source**: `gojb/zurich_airport_hires.r16`
- **Main script**: `gojb/node_3d.gd`
- **Building loader**: `gojb/building_loader.gd`
- **JSBSim binding**: `src/jsbgodot.cpp`, `src/jsbgodot.h`
