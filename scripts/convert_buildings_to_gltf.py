#!/usr/bin/env python3
"""
Convert SwissTopo swissBUILDINGS3D GDB files to Godot-compatible GLTF format.

This script reads 3D building data from Swiss GDB files and converts them to 
GLTF format that can be imported into Godot. It handles:
- Coordinate transformation from Swiss LV95 (EPSG:2056) to local Godot coordinates
- TIN (Triangulated Irregular Network) geometry conversion to triangle meshes
- Merging building components (walls, roof, floor) into single meshes
- Optional LOD generation for performance

Usage:
    python3 convert_buildings_to_gltf.py <gdb_file_or_directory> [options]
    
Example:
    python3 convert_buildings_to_gltf.py gojb/buildings_data/test_tile/*.gdb --output gojb/Assets/buildings
"""

import argparse
import json
import struct
import sys
from pathlib import Path
from typing import Optional

import fiona
import numpy as np
from pyproj import Transformer
from shapely.geometry import shape


# Swiss terrain CENTER in LV95 coordinates
# Based on observed terrain edges with zurich_airport_hires.r16 at vertex_spacing=1.0
# The 0.5m resolution heightmap displays as 8km x 8km (2x scale)
# Observed terrain center: E 2686872, N 1257719
# Note: +X in Godot = West, +Z in Godot = North
TERRAIN_CENTER_E = 2686872.0  # Easting at Godot X=0
TERRAIN_CENTER_N = 1257719.0  # Northing at Godot Z=0


def parse_tin_geometry(geometry) -> tuple:
    """
    Parse TIN (Triangulated Irregular Network) geometry into vertices and triangles.
    
    TIN geometry in GDB is stored as MultiPolygon Z where each polygon is a triangle.
    """
    vertices = []
    triangles = []
    vertex_map = {}  # Map (x,y,z) -> vertex index for deduplication
    
    geom = shape(geometry)
    
    if geom.geom_type == 'MultiPolygon':
        for polygon in geom.geoms:
            # Each polygon in TIN should be a triangle
            coords = list(polygon.exterior.coords)
            if len(coords) >= 4:  # Triangle has 4 coords (first repeated at end)
                tri_indices = []
                for coord in coords[:3]:  # Only first 3 points
                    x, y, z = coord[0], coord[1], coord[2] if len(coord) > 2 else 0.0
                    key = (round(x, 4), round(y, 4), round(z, 4))
                    
                    if key not in vertex_map:
                        vertex_map[key] = len(vertices)
                        vertices.append([x, y, z])
                    
                    tri_indices.append(vertex_map[key])
                
                if len(tri_indices) == 3:
                    triangles.append(tri_indices)
                    
    elif geom.geom_type == 'Polygon':
        coords = list(geom.exterior.coords)
        if len(coords) >= 4:
            tri_indices = []
            for coord in coords[:3]:
                x, y, z = coord[0], coord[1], coord[2] if len(coord) > 2 else 0.0
                key = (round(x, 4), round(y, 4), round(z, 4))
                
                if key not in vertex_map:
                    vertex_map[key] = len(vertices)
                    vertices.append([x, y, z])
                
                tri_indices.append(vertex_map[key])
            
            if len(tri_indices) == 3:
                triangles.append(tri_indices)
    
    return np.array(vertices, dtype=np.float32), np.array(triangles, dtype=np.uint32)


def transform_to_godot_coords(vertices: np.ndarray, origin_e: float, origin_n: float) -> np.ndarray:
    """
    Transform Swiss LV95 coordinates to Godot local coordinates.
    
    Swiss LV95 (E, N, Z) -> Godot (X, Y, Z):
    - E (Easting) -> -X (with offset from terrain origin, flipped so +X = East)
    - N (Northing) -> Z (with offset, +Z = North) 
    - Z (Height ASL) -> Y (Godot up axis)
    
    Note: We use E - origin (not origin - E) to avoid mirroring the geometry.
    The terrain and building loader both expect +X = West, so we negate.
    """
    if len(vertices) == 0:
        return vertices
    
    transformed = np.zeros_like(vertices)
    # Keep consistent with terrain: +X = West means X = origin - E
    # But to avoid mirroring buildings, we use -(E - origin) = origin - E
    # which is equivalent but we need to flip triangle winding
    transformed[:, 0] = origin_e - vertices[:, 0]      # X = origin - E (+X = West)
    transformed[:, 1] = vertices[:, 2]                  # Y = Z (height)
    transformed[:, 2] = vertices[:, 1] - origin_n      # Z = N - origin (+Z = North)
    
    return transformed


def flip_triangle_winding(triangles: np.ndarray) -> np.ndarray:
    """
    Flip triangle winding order (reverse vertex order in each triangle).
    This is needed when a coordinate axis is inverted to maintain correct face normals.
    """
    if len(triangles) == 0:
        return triangles
    # Swap second and third vertex indices to reverse winding
    flipped = triangles.copy()
    flipped[:, 1], flipped[:, 2] = triangles[:, 2].copy(), triangles[:, 1].copy()
    return flipped


def compute_normals(vertices: np.ndarray, triangles: np.ndarray) -> np.ndarray:
    """Compute per-vertex normals by averaging face normals."""
    normals = np.zeros_like(vertices)
    
    for tri in triangles:
        v0, v1, v2 = vertices[tri[0]], vertices[tri[1]], vertices[tri[2]]
        edge1 = v1 - v0
        edge2 = v2 - v0
        face_normal = np.cross(edge1, edge2)
        norm = np.linalg.norm(face_normal)
        if norm > 0:
            face_normal /= norm
            for idx in tri:
                normals[idx] += face_normal
    
    # Normalize
    norms = np.linalg.norm(normals, axis=1, keepdims=True)
    norms[norms == 0] = 1  # Avoid division by zero
    normals /= norms
    
    return normals.astype(np.float32)


def create_gltf_buffer(vertices: np.ndarray, normals: np.ndarray, triangles: np.ndarray) -> bytes:
    """Create a binary buffer containing vertex, normal, and index data."""
    buffer = b''
    
    # Vertices (12 bytes per vertex: 3 x float32)
    buffer += vertices.astype(np.float32).tobytes()
    
    # Normals (12 bytes per normal: 3 x float32)
    buffer += normals.astype(np.float32).tobytes()
    
    # Indices (4 bytes per index: uint32, or 2 bytes if < 65536 vertices)
    if len(vertices) < 65536:
        buffer += triangles.flatten().astype(np.uint16).tobytes()
    else:
        buffer += triangles.flatten().astype(np.uint32).tobytes()
    
    return buffer


def create_gltf(vertices: np.ndarray, normals: np.ndarray, triangles: np.ndarray, 
                name: str = "Building") -> dict:
    """Create a GLTF 2.0 JSON structure."""
    
    if len(vertices) == 0 or len(triangles) == 0:
        return None
    
    # Calculate bounds
    v_min = vertices.min(axis=0).tolist()
    v_max = vertices.max(axis=0).tolist()
    
    # Buffer layout
    vertex_byte_length = len(vertices) * 12  # 3 floats * 4 bytes
    normal_byte_length = len(normals) * 12
    
    use_uint16 = len(vertices) < 65536
    index_byte_length = len(triangles) * 3 * (2 if use_uint16 else 4)
    
    total_byte_length = vertex_byte_length + normal_byte_length + index_byte_length
    
    gltf = {
        "asset": {
            "version": "2.0",
            "generator": "SwissTopo Buildings Converter"
        },
        "scene": 0,
        "scenes": [{"nodes": [0]}],
        "nodes": [
            {
                "mesh": 0,
                "name": name
            }
        ],
        "meshes": [
            {
                "name": name,
                "primitives": [
                    {
                        "attributes": {
                            "POSITION": 0,
                            "NORMAL": 1
                        },
                        "indices": 2,
                        "material": 0
                    }
                ]
            }
        ],
        "materials": [
            {
                "name": "BuildingMaterial",
                "pbrMetallicRoughness": {
                    "baseColorFactor": [0.8, 0.75, 0.7, 1.0],
                    "metallicFactor": 0.0,
                    "roughnessFactor": 0.8
                }
            }
        ],
        "accessors": [
            {
                "bufferView": 0,
                "byteOffset": 0,
                "componentType": 5126,  # FLOAT
                "count": len(vertices),
                "type": "VEC3",
                "min": v_min,
                "max": v_max
            },
            {
                "bufferView": 1,
                "byteOffset": 0,
                "componentType": 5126,  # FLOAT
                "count": len(normals),
                "type": "VEC3"
            },
            {
                "bufferView": 2,
                "byteOffset": 0,
                "componentType": 5123 if use_uint16 else 5125,  # UNSIGNED_SHORT or UNSIGNED_INT
                "count": len(triangles) * 3,
                "type": "SCALAR"
            }
        ],
        "bufferViews": [
            {
                "buffer": 0,
                "byteOffset": 0,
                "byteLength": vertex_byte_length,
                "target": 34962  # ARRAY_BUFFER
            },
            {
                "buffer": 0,
                "byteOffset": vertex_byte_length,
                "byteLength": normal_byte_length,
                "target": 34962  # ARRAY_BUFFER
            },
            {
                "buffer": 0,
                "byteOffset": vertex_byte_length + normal_byte_length,
                "byteLength": index_byte_length,
                "target": 34963  # ELEMENT_ARRAY_BUFFER
            }
        ],
        "buffers": [
            {
                "byteLength": total_byte_length
            }
        ]
    }
    
    return gltf


def write_glb(gltf: dict, buffer: bytes, output_path: Path):
    """Write a binary GLTF (GLB) file."""
    json_str = json.dumps(gltf, separators=(',', ':'))
    json_bytes = json_str.encode('utf-8')
    
    # Pad JSON to 4-byte alignment
    json_padding = (4 - len(json_bytes) % 4) % 4
    json_bytes += b' ' * json_padding
    
    # Pad buffer to 4-byte alignment
    buffer_padding = (4 - len(buffer) % 4) % 4
    buffer += b'\x00' * buffer_padding
    
    # Update buffer URI to indicate embedded
    gltf['buffers'][0]['byteLength'] = len(buffer) - buffer_padding
    
    # Recalculate JSON
    json_str = json.dumps(gltf, separators=(',', ':'))
    json_bytes = json_str.encode('utf-8')
    json_padding = (4 - len(json_bytes) % 4) % 4
    json_bytes += b' ' * json_padding
    
    # GLB header (12 bytes)
    # Magic: "glTF"
    # Version: 2
    # Length: total file size
    total_length = 12 + 8 + len(json_bytes) + 8 + len(buffer)
    
    with open(output_path, 'wb') as f:
        # Header
        f.write(b'glTF')                          # magic
        f.write(struct.pack('<I', 2))             # version
        f.write(struct.pack('<I', total_length))  # length
        
        # JSON chunk
        f.write(struct.pack('<I', len(json_bytes)))  # chunk length
        f.write(b'JSON')                              # chunk type
        f.write(json_bytes)
        
        # BIN chunk
        f.write(struct.pack('<I', len(buffer)))   # chunk length
        f.write(b'BIN\x00')                       # chunk type
        f.write(buffer)


def process_gdb_file(gdb_path: Path, output_dir: Path, origin_e: float, origin_n: float,
                    layer_name: str = "Building_solid", max_buildings: Optional[int] = None):
    """Process a single GDB file and export buildings to GLB."""
    
    print(f"\nProcessing: {gdb_path}")
    
    # List available layers
    layers = fiona.listlayers(str(gdb_path))
    print(f"  Available layers: {layers}")
    
    if layer_name not in layers:
        print(f"  WARNING: Layer '{layer_name}' not found, trying alternatives...")
        for alt in ["Building_solid", "Roof_solid", "Wall"]:
            if alt in layers:
                layer_name = alt
                print(f"  Using layer: {layer_name}")
                break
        else:
            print(f"  ERROR: No suitable layer found")
            return 0
    
    # Read buildings
    all_vertices = []
    all_triangles = []
    vertex_offset = 0
    building_count = 0
    
    with fiona.open(str(gdb_path), layer=layer_name) as src:
        print(f"  Reading {len(src)} features from '{layer_name}'...")
        
        for i, feature in enumerate(src):
            if max_buildings and i >= max_buildings:
                break
            
            geom = feature.get('geometry')
            if geom is None:
                continue
            
            vertices, triangles = parse_tin_geometry(geom)
            
            if len(vertices) > 0 and len(triangles) > 0:
                # Transform coordinates
                vertices = transform_to_godot_coords(vertices, origin_e, origin_n)
                
                # Flip triangle winding to correct for X-axis inversion
                triangles = flip_triangle_winding(triangles)
                
                # Offset triangle indices
                triangles = triangles + vertex_offset
                
                all_vertices.append(vertices)
                all_triangles.append(triangles)
                vertex_offset += len(vertices)
                building_count += 1
            
            if (i + 1) % 100 == 0:
                print(f"    Processed {i + 1} features...")
    
    if building_count == 0:
        print("  No valid buildings found")
        return 0
    
    # Merge all geometry
    print(f"  Merging {building_count} buildings...")
    vertices = np.vstack(all_vertices)
    triangles = np.vstack(all_triangles)
    
    print(f"  Computing normals...")
    normals = compute_normals(vertices, triangles)
    
    # Create GLTF
    tile_name = gdb_path.stem.replace("swissBUILDINGS3D_3-0_", "buildings_")
    gltf = create_gltf(vertices, normals, triangles, name=tile_name)
    
    if gltf is None:
        print("  ERROR: Failed to create GLTF")
        return 0
    
    # Create buffer
    buffer = create_gltf_buffer(vertices, normals, triangles)
    
    # Write GLB
    output_dir.mkdir(parents=True, exist_ok=True)
    output_path = output_dir / f"{tile_name}.glb"
    write_glb(gltf, buffer, output_path)
    
    file_size_mb = output_path.stat().st_size / (1024 * 1024)
    print(f"  Written: {output_path} ({file_size_mb:.2f} MB)")
    print(f"  Vertices: {len(vertices):,}, Triangles: {len(triangles):,}")
    
    return building_count


def main():
    parser = argparse.ArgumentParser(description="Convert SwissTopo buildings to GLTF")
    parser.add_argument("input", type=Path, help="GDB file or directory containing GDB files")
    parser.add_argument("--output", "-o", type=Path, 
                       default=Path(__file__).parent.parent / "gojb" / "Assets" / "buildings",
                       help="Output directory for GLB files")
    parser.add_argument("--origin-e", type=float, default=TERRAIN_CENTER_E,
                       help=f"Terrain center Easting (default: {TERRAIN_CENTER_E})")
    parser.add_argument("--origin-n", type=float, default=TERRAIN_CENTER_N,
                       help=f"Terrain center Northing (default: {TERRAIN_CENTER_N})")
    parser.add_argument("--layer", default="Building_solid",
                       help="Layer name to extract (default: Building_solid)")
    parser.add_argument("--max-buildings", type=int, default=None,
                       help="Maximum buildings per tile (for testing)")
    args = parser.parse_args()
    
    print("=" * 60)
    print("SwissBUILDINGS3D to GLTF Converter")
    print("=" * 60)
    print(f"Input: {args.input}")
    print(f"Output: {args.output}")
    print(f"Origin: E={args.origin_e}, N={args.origin_n}")
    
    # Find GDB files (GDB is a directory, not a file)
    gdb_files = []
    if args.input.is_dir():
        if args.input.suffix == ".gdb" or args.input.name.endswith(".gdb"):
            # Input is a GDB directory itself
            gdb_files = [args.input]
        else:
            # Search for GDB directories within input
            gdb_files = [p for p in args.input.rglob("*.gdb") if p.is_dir()]
    else:
        print(f"ERROR: Input must be a .gdb directory or directory containing .gdb folders")
        return 1
    
    if not gdb_files:
        print("ERROR: No GDB files found")
        return 1
    
    print(f"Found {len(gdb_files)} GDB files")
    
    total_buildings = 0
    for gdb_path in sorted(gdb_files):
        count = process_gdb_file(
            gdb_path, args.output, 
            args.origin_e, args.origin_n,
            args.layer, args.max_buildings
        )
        total_buildings += count
    
    print("\n" + "=" * 60)
    print(f"COMPLETE: Converted {total_buildings} buildings from {len(gdb_files)} tiles")
    print("=" * 60)
    print("\nTo use in Godot:")
    print("1. Import the .glb files into your project")
    print("2. Create instances or use MultiMeshInstance3D for performance")
    print("3. Position should match terrain (same coordinate origin)")
    
    return 0


if __name__ == "__main__":
    sys.exit(main())
