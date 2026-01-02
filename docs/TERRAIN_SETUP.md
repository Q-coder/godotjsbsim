# Terrain Setup and JSBSim Collision Detection

This document describes how to import terrain data and how the collision detection between Godot's Terrain3D and JSBSim works.

## Overview

The flight simulator uses:
- **Godot Terrain3D** for visual terrain rendering and collision queries
- **JSBSim** for flight dynamics and ground reaction physics
- **SwissTopo swissALTI3D** for high-resolution elevation data (0.5m resolution)
- **SwissTopo swissBUILDINGS3D** for 3D building models

These systems need to share terrain elevation data so the aircraft can properly interact with the ground.

## Current Terrain: Schaffhausen Region (8km Extended)

The active terrain covers the Schaffhausen area in northern Switzerland:

| Parameter | Value |
|-----------|-------|
| **Center** | E 2,685,750 / N 1,282,500 |
| **Size** | 8 km × 8 km |
| **Resolution** | 1m per pixel (8000×8000) |
| **Elevation Range** | 422m - 799m ASL |
| **File** | `schaffhausen_8km_extended.r16` |

### Terrain Bounds (Swiss LV95)
- **West**: E 2,681,750
- **East**: E 2,689,750
- **South**: N 1,278,500
- **North**: N 1,286,500

### Coverage
- ✅ Schmerlat Airfield (E 2,681,968) - 218m from west edge
- ✅ Hallau wine village
- ✅ Schaffhausen city edge (E 2,689,500) - 250m from east edge

### Terrain3D Import Settings

| Setting | Value |
|---------|-------|
| **File** | `schaffhausen_8km_extended.r16` |
| **R16 Size** | 8000 |
| **Init Position** | `Vector2i(-4000, -4000)` |
| **r16_range** | `Vector2(0, 810)` |
| **Height Offset** | `0` |
| **vertex_spacing** | `1.0` |

### Previous Terrains (Archived)

| Parameter | Current 8km Extended | Previous 12km (Failed) | Original 8km |
|-----------|---------------------|----------------------|--------------|
| **Center** | E 2,685,750 / N 1,282,500 | E 2,684,000 / N 1,282,500 | E 2,681,968 / N 1,282,869 |
| **Size** | 8×8 km | 12×12 km | 8×8 km |
| **File** | `schaffhausen_8km_extended.r16` | `schaffhausen_extended.r16` | `schaffhausen.r16` |
| **Status** | Active | Exceeds Terrain3D position limit | Archived |

**Note**: Terrain3D has a position limit of ±4096, so maximum terrain size with init position at corner is 8192×8192 pixels.

## Terrain Processing Pipeline

### 1. Download SwissTopo Tiles

Use the `scripts/process_swisstopo_terrain.py` script:

```bash
# Step 1: Generate CSV of required tiles
python scripts/process_swisstopo_terrain.py \
    --center-e 2681968 --center-n 1282869 \
    --size 8000 \
    --list-tiles --output tiles.csv

# Step 2: Download tiles from SwissTopo
python scripts/process_swisstopo_terrain.py \
    --center-e 2681968 --center-n 1282869 \
    --size 8000 \
    --download --output downloads/

# Step 3: Process and merge into R16
python scripts/process_swisstopo_terrain.py \
    --center-e 2681968 --center-n 1282869 \
    --size 8000 \
    --process downloads/ --output gojb/schaffhausen.r16
```

### 2. Critical Processing Steps

**IMPORTANT**: SwissTopo GeoTIFF tiles have opposite orientation to Godot/Terrain3D:

```python
# Both flips are REQUIRED for correct orientation
terrain_data = np.flipud(terrain_data)  # Flip North-South
terrain_data = np.fliplr(terrain_data)  # Flip East-West
```

Without these flips:
- North appears as South
- East appears as West
- Landmarks will be in wrong positions

### 3. Import into Terrain3D

1. Open Godot and select the **Terrain3D** node
2. Go to **Terrain3D → Tools → Import**
3. Configure settings:

| Setting | Value | Notes |
|---------|-------|-------|
| **File** | `schaffhausen.r16` | Your heightmap file |
| **r16_range** | `Vector2(0, 810)` | Calculated from elevation range |
| **Height Offset** | `0` | Godot Y = real ASL |
| **Height Scale** | `1.0` | Real-world meters |
| **vertex_spacing** | `1.0` | 1 meter per pixel |

**R16 Range Calculation**:
```
r16_range.y = max_elevation - min_elevation + small_buffer
Example: 806 - 402 + 6 = 410, round up to safe value like 810
```

## Building Import

### 1. Download Building Tiles

Buildings come from SwissTopo swissBUILDINGS3D 3.0 in GDB (Geodatabase) format:

```bash
# Convert GDB to GLB format
python scripts/convert_buildings_to_gltf.py \
    --input buildings_gdb/ \
    --output gojb/Assets/buildings_schaffhausen/ \
    --origin-e 2681968 --origin-n 1282869
```

The `--origin-e` and `--origin-n` must match the terrain center!

### 2. Building Loader Configuration

In `building_loader.gd`:
- Set `terrain_area` to the correct region
- Update `BUILDING_TILES_*` dictionary with tile definitions
- Each tile has: `id`, `path`, `center` (Godot coordinates)

## Aircraft Model Scale

**IMPORTANT**: The Cessna 172 model is in decimeters, not meters!

| Model | Raw Size | Scaled (0.1x) | Real |
|-------|----------|---------------|------|
| Wingspan | 1110 units | 111m → 11.1m | 10.97m |
| Length | 799 units | 79.9m → 8.0m | 8.28m |
| Height | 318 units | 31.8m → 3.2m | 2.72m |

Apply scale factor `0.1` in `C172p.tscn`:
```
transform = Transform3D(0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.1, 0, 0, 0)
```

## Coordinate Systems

### Swiss LV95 to Godot World

The terrain is centered at Swiss LV95 coordinates, converted to Godot:
- **Terrain Center E**: 2681968.0 (Easting at Godot X=0)
- **Terrain Center N**: 1282869.0 (Northing at Godot Z=0)

Conversion formulas:
```
Godot X = TERRAIN_CENTER_E - Swiss_E  (+X = West)
Godot Z = Swiss_N - TERRAIN_CENTER_N  (+Z = North)
```

### Godot to WGS84 (Lat/Lon)

The HUD displays real-world coordinates by:
1. Converting Godot position back to Swiss LV95
2. Using swisstopo approximate formulas to convert LV95 → WGS84

```gdscript
func lv95_to_wgs84(easting: float, northing: float) -> Vector2:
    # Returns Vector2(latitude, longitude) in degrees
```

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

## Configuration Variables

### In node_3d.gd

| Variable | Value | Purpose |
|----------|-------|---------|
| `WHEEL_GROUND_CLEARANCE` | `0.5` | Meters to lift visual model to prevent wheel clipping |
| `TERRAIN_CENTER_E` | `2681968.0` | Swiss Easting at Godot X=0 (Schmerlat) |
| `TERRAIN_CENTER_N` | `1282869.0` | Swiss Northing at Godot Z=0 (Schmerlat) |

### In JSBGodot C++

| Variable | Purpose |
|----------|---------|
| `godot_terrain_y_offset` | Small offset (e.g., 0.5m) to lift visual model above ground |

### Orbit Camera Settings

| Variable | Value | Purpose |
|----------|-------|---------|
| `orbit_distance` | `6.0` | Default camera distance (meters) |
| `ORBIT_DISTANCE_MIN` | `1.5` | Minimum zoom distance |
| `ORBIT_DISTANCE_MAX` | `50.0` | Maximum zoom distance |

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
- Ensure building origin coordinates match terrain center
- Check that `--origin-e` and `--origin-n` in convert script match terrain

## Lessons Learned

### SwissTopo Data Orientation
SwissTopo GeoTIFF files have Y-axis (rows) going from North to South, and X-axis from West to East. Godot/Terrain3D expects the opposite. **Both `np.flipud()` and `np.fliplr()` are required**.

### Model Scale Units
3D models from various sources may use different units:
- **Meters**: 1 unit = 1 meter (preferred)
- **Decimeters**: 1 unit = 0.1 meters (Cessna 172 model uses this)
- **Centimeters**: 1 unit = 0.01 meters

Always check model dimensions against real-world specs and apply appropriate scale.

### Verification Strategy
When importing new terrain, use landmarks you know well to verify orientation and scale. The Schaffhausen region was chosen because the user knows the local topography.

## File Locations

- **Terrain data**: `gojb/Terra3DData/` (Terrain3D .res files)
- **Heightmap source**: `gojb/schaffhausen.r16`
- **Main script**: `gojb/node_3d.gd`
- **Building loader**: `gojb/building_loader.gd`
- **Building GLBs**: `gojb/Assets/buildings_schaffhausen/`
- **Cessna model**: `gojb/Assets/cessna172/C172P_1.blend`
- **JSBSim binding**: `src/jsbgodot.cpp`, `src/jsbgodot.h`
- **Terrain processing**: `scripts/process_swisstopo_terrain.py`
- **Building conversion**: `scripts/convert_buildings_to_gltf.py`
