#!/usr/bin/env python3
"""
Process SwissTopo swissALTI3D tiles into a single R16 heightmap for Godot/Terrain3D.

This script:
1. Downloads required GeoTIFF tiles from SwissTopo
2. Merges them into a single GeoTIFF
3. Converts to R16 format for Terrain3D

Usage:
    python process_swisstopo_terrain.py --bounds 2682872 1253719 2690872 1261719 --output zurich_airport.r16

Requirements:
    pip install rasterio numpy requests tqdm
"""

import argparse
import os
import sys
import tempfile
from pathlib import Path

try:
    import numpy as np
    import rasterio
    from rasterio.merge import merge
    from rasterio.enums import Resampling
    import requests
    from tqdm import tqdm
except ImportError as e:
    print(f"Missing required package: {e}")
    print("Install with: pip install rasterio numpy requests tqdm")
    sys.exit(1)


def download_tile(url: str, output_dir: Path) -> Path:
    """Download a single tile if not already cached."""
    filename = url.split('/')[-1]
    output_path = output_dir / filename
    
    if output_path.exists():
        return output_path
    
    response = requests.get(url, stream=True)
    response.raise_for_status()
    
    with open(output_path, 'wb') as f:
        for chunk in response.iter_content(chunk_size=8192):
            f.write(chunk)
    
    return output_path


def get_tile_urls_for_bounds(csv_path: str, west: int, south: int, east: int, north: int) -> list:
    """Filter tile URLs that fall within the specified bounds."""
    import re
    
    urls = []
    with open(csv_path, 'r') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            
            # Extract coordinates from filename like swissalti3d_2020_2686-1257
            match = re.search(r'swissalti3d_\d+_(\d+)-(\d+)', line)
            if match:
                tile_e = int(match.group(1)) * 1000  # SW corner of tile
                tile_n = int(match.group(2)) * 1000
                
                # Check if tile overlaps with bounds
                # Tile is 1km x 1km
                if (tile_e < east and tile_e + 1000 > west and
                    tile_n < north and tile_n + 1000 > south):
                    urls.append(line)
    
    return urls


def merge_geotiffs(tif_paths: list, output_path: Path, bounds: tuple = None):
    """Merge multiple GeoTIFFs into one."""
    datasets = [rasterio.open(p) for p in tif_paths]
    
    # Merge all tiles
    mosaic, out_transform = merge(datasets)
    
    # Get metadata from first file
    out_meta = datasets[0].meta.copy()
    out_meta.update({
        "driver": "GTiff",
        "height": mosaic.shape[1],
        "width": mosaic.shape[2],
        "transform": out_transform,
    })
    
    # Close input datasets
    for ds in datasets:
        ds.close()
    
    # Write merged file
    with rasterio.open(output_path, "w", **out_meta) as dest:
        dest.write(mosaic)
    
    return output_path


def crop_to_bounds(input_path: Path, output_path: Path, west: int, south: int, east: int, north: int):
    """Crop GeoTIFF to exact bounds."""
    from rasterio.windows import from_bounds
    
    with rasterio.open(input_path) as src:
        window = from_bounds(west, south, east, north, src.transform)
        
        # Read the windowed data
        data = src.read(1, window=window)
        
        # Calculate new transform for the window
        new_transform = src.window_transform(window)
        
        # Update metadata
        out_meta = src.meta.copy()
        out_meta.update({
            "height": data.shape[0],
            "width": data.shape[1],
            "transform": new_transform,
        })
        
        with rasterio.open(output_path, "w", **out_meta) as dest:
            dest.write(data, 1)
    
    return output_path


def geotiff_to_r16(input_path: Path, output_path: Path, target_size: int = None):
    """Convert GeoTIFF to R16 format for Terrain3D."""
    with rasterio.open(input_path) as src:
        data = src.read(1)
        
        print(f"\n=== GeoTIFF Info ===")
        print(f"  Size: {data.shape[1]} x {data.shape[0]}")
        print(f"  Bounds: {src.bounds}")
        print(f"  CRS: {src.crs}")
        print(f"  Resolution: {src.res}")
        
        # Elevation statistics
        valid = data[data != src.nodata] if src.nodata else data[data > 0]
        min_elev = float(valid.min())
        max_elev = float(valid.max())
        mean_elev = float(valid.mean())
        print(f"\n=== Elevation Statistics ===")
        print(f"  Min: {min_elev:.2f} m")
        print(f"  Max: {max_elev:.2f} m")
        print(f"  Mean: {mean_elev:.2f} m")
        
        # Resample if target size specified
        if target_size and (data.shape[0] != target_size or data.shape[1] != target_size):
            print(f"\n  Resampling from {data.shape[1]}x{data.shape[0]} to {target_size}x{target_size}")
            from scipy.ndimage import zoom
            factor_y = target_size / data.shape[0]
            factor_x = target_size / data.shape[1]
            data = zoom(data, (factor_y, factor_x), order=1)
            # Recalculate stats after resampling
            valid = data[data > 0]
            min_elev = float(valid.min())
            max_elev = float(valid.max())
        
        # For R16, we need to encode elevations into uint16 (0-65535)
        # Terrain3D decodes as: elevation = raw / 65535 * (r16_range.y - r16_range.x) + r16_range.x
        # 
        # We'll use r16_range = (0, max_elev_rounded) where raw 65535 = max elevation
        # This gives us full 16-bit precision for the elevation range
        
        # Round max up to nearest 10m for cleaner r16_range value
        r16_max = np.ceil(max_elev / 10) * 10
        
        # Encode: raw = elevation / r16_max * 65535
        data_r16 = (data / r16_max * 65535).astype(np.uint16)
        
        # Handle nodata (set to 0)
        if src.nodata:
            data_r16[data == src.nodata] = 0
        
        # Flip vertically: GeoTIFF has row 0 at north, but Terrain3D R16 expects row 0 at south
        data_r16 = np.flipud(data_r16)
        # Flip horizontally: Match coordinate system where godot_x = center_e - swiss_e
        data_r16 = np.fliplr(data_r16)
        
        print(f"\n=== R16 Output ===")
        print(f"  Size: {data_r16.shape[1]} x {data_r16.shape[0]}")
        print(f"  Raw range: {data_r16.min()} - {data_r16.max()}")
        print(f"  r16_range for Terrain3D: Vector2(0, {r16_max:.2f})")
        print(f"  Elevation range: {min_elev:.2f}m - {max_elev:.2f}m")
        print(f"  Note: Array flipped vertically and horizontally for Terrain3D import")
        
        # Write R16 (raw uint16, row-major)
        data_r16.tofile(output_path)
        print(f"\n  Written to: {output_path}")
        
        return r16_max  # Return max for r16_range


def main():
    parser = argparse.ArgumentParser(description='Process SwissTopo terrain tiles')
    parser.add_argument('--csv', required=True, help='Path to SwissTopo download CSV')
    parser.add_argument('--bounds', nargs=4, type=int, required=True,
                        metavar=('WEST', 'SOUTH', 'EAST', 'NORTH'),
                        help='Bounds in LV95 coordinates (e.g., 2682872 1253719 2690872 1261719)')
    parser.add_argument('--output', required=True, help='Output R16 file path')
    parser.add_argument('--cache-dir', default='./terrain_cache', help='Directory to cache downloaded tiles')
    parser.add_argument('--target-size', type=int, help='Resample to square size (e.g., 8192)')
    
    args = parser.parse_args()
    
    west, south, east, north = args.bounds
    cache_dir = Path(args.cache_dir)
    cache_dir.mkdir(exist_ok=True)
    
    print(f"=== SwissTopo Terrain Processor ===")
    print(f"Bounds: E {west:,} - {east:,}, N {south:,} - {north:,}")
    print(f"Size: {east-west}m x {north-south}m")
    
    # Get tile URLs for bounds
    urls = get_tile_urls_for_bounds(args.csv, west, south, east, north)
    print(f"\nFound {len(urls)} tiles to download")
    
    if not urls:
        print("ERROR: No tiles found for the specified bounds!")
        return 1
    
    # Download tiles
    print("\nDownloading tiles...")
    tif_paths = []
    for url in tqdm(urls, desc="Downloading"):
        path = download_tile(url, cache_dir)
        tif_paths.append(path)
    
    # Merge tiles
    print("\nMerging tiles...")
    merged_path = cache_dir / "merged.tif"
    merge_geotiffs(tif_paths, merged_path)
    
    # Crop to exact bounds
    print("\nCropping to bounds...")
    cropped_path = cache_dir / "cropped.tif"
    crop_to_bounds(merged_path, cropped_path, west, south, east, north)
    
    # Convert to R16
    print("\nConverting to R16...")
    max_elev = geotiff_to_r16(cropped_path, Path(args.output), args.target_size)
    
    print(f"\n=== DONE ===")
    print(f"R16 file: {args.output}")
    print(f"\nIn Terrain3D, use:")
    print(f"  r16_range = Vector2(0, {max_elev:.2f})")
    print(f"  height_offset = 0")
    
    return 0


if __name__ == '__main__':
    sys.exit(main())
