extends Node3D
## BuildingLoader - Dynamically loads SwissTopo building tiles based on camera position
##
## This script manages loading/unloading of building GLB files based on distance
## from the aircraft. Buildings are organized in tiles that cover roughly 3x3 km each.
##
## Usage:
##   1. Add this script to a Node3D in your scene
##   2. Set the aircraft_node to track
##   3. Optionally adjust load_distance and unload_distance

class_name BuildingLoader

## The node to track for position (usually the aircraft)
@export var aircraft_node: Node3D

## Distance (in meters) at which to load building tiles
@export var load_distance: float = 8000.0

## Distance (in meters) at which to unload building tiles (should be > load_distance)
@export var unload_distance: float = 12000.0

## Maximum number of tiles to load per frame (to prevent stuttering)
@export var max_loads_per_frame: int = 1

## Enable debug visualization
@export var debug_mode: bool = false

## Enable/disable building loading entirely
@export var enabled: bool = true

## Position offset for fine-tuning building alignment (X, Y, Z)
## Buildings use their original Swiss ASL elevations
## This offset is applied on top for minor adjustments if needed
@export var building_offset: Vector3 = Vector3(0, 0, 0)

## Roof color (terracotta/brownish-red)
@export var roof_color: Color = Color(0.72, 0.38, 0.28, 1.0)

## Wall color (warm beige/cream)
@export var wall_color: Color = Color(0.92, 0.87, 0.75, 1.0)

## Building tile information
## Each tile covers approximately 2.5x2.5 km in Swiss coordinates
## Coordinates are in Godot world space (already transformed from Swiss LV95)
## Coordinate system: +X = West, +Z = North

## Zurich Airport tiles (terrain center: E 2686872, N 1257719)
## Centers calculated from Swiss LV95 tile coordinates using:
##   Godot X = TERRAIN_CENTER_E - Swiss_E
##   Godot Z = Swiss_N - TERRAIN_CENTER_N
const BUILDING_TILES_ZURICH: Array[Dictionary] = [
	{"id": "1071-12", "center": Vector3(1067, 0, 1502), "path": "res://Assets/buildings/buildings_1071-12.glb"},
	{"id": "1071-14", "center": Vector3(1064, 0, -1505), "path": "res://Assets/buildings/buildings_1071-14.glb"},
	{"id": "1071-21", "center": Vector3(5442, 0, 1488), "path": "res://Assets/buildings/buildings_1071-21.glb"},
	{"id": "1071-22", "center": Vector3(9806, 0, 1487), "path": "res://Assets/buildings/buildings_1071-22.glb"},
	{"id": "1071-23", "center": Vector3(5405, 0, -1456), "path": "res://Assets/buildings/buildings_1071-23.glb"},
	{"id": "1071-24", "center": Vector3(9840, 0, -1498), "path": "res://Assets/buildings/buildings_1071-24.glb"},
	{"id": "1071-32", "center": Vector3(1060, 0, -4479), "path": "res://Assets/buildings/buildings_1071-32.glb"},
	{"id": "1071-34", "center": Vector3(1045, 0, -7565), "path": "res://Assets/buildings/buildings_1071-34.glb"},
	{"id": "1071-41", "center": Vector3(5426, 0, -4512), "path": "res://Assets/buildings/buildings_1071-41.glb"},
	{"id": "1071-42", "center": Vector3(9817, 0, -4507), "path": "res://Assets/buildings/buildings_1071-42.glb"},
	{"id": "1071-43", "center": Vector3(5463, 0, -7495), "path": "res://Assets/buildings/buildings_1071-43.glb"},
	{"id": "1071-44", "center": Vector3(9812, 0, -7474), "path": "res://Assets/buildings/buildings_1071-44.glb"},
	{"id": "1072-11", "center": Vector3(14184, 0, 1491), "path": "res://Assets/buildings/buildings_1072-11.glb"},
	{"id": "1072-12", "center": Vector3(18566, 0, 1498), "path": "res://Assets/buildings/buildings_1072-12.glb"},
	{"id": "1072-13", "center": Vector3(14235, 0, -1494), "path": "res://Assets/buildings/buildings_1072-13.glb"},
	{"id": "1072-14", "center": Vector3(18567, 0, -1493), "path": "res://Assets/buildings/buildings_1072-14.glb"},
	{"id": "1072-31", "center": Vector3(14170, 0, -4499), "path": "res://Assets/buildings/buildings_1072-31.glb"},
	{"id": "1072-32", "center": Vector3(18564, 0, -4501), "path": "res://Assets/buildings/buildings_1072-32.glb"},
	{"id": "1072-33", "center": Vector3(14186, 0, -7495), "path": "res://Assets/buildings/buildings_1072-33.glb"},
	{"id": "1072-34", "center": Vector3(18493, 0, -7509), "path": "res://Assets/buildings/buildings_1072-34.glb"},
	{"id": "1091-12", "center": Vector3(1063, 0, -10506), "path": "res://Assets/buildings/buildings_1091-12.glb"},
	{"id": "1091-21", "center": Vector3(5435, 0, -10496), "path": "res://Assets/buildings/buildings_1091-21.glb"},
	{"id": "1091-22", "center": Vector3(9827, 0, -10510), "path": "res://Assets/buildings/buildings_1091-22.glb"},
	{"id": "1092-11", "center": Vector3(14194, 0, -10501), "path": "res://Assets/buildings/buildings_1092-11.glb"},
	{"id": "1092-12", "center": Vector3(18532, 0, -10491), "path": "res://Assets/buildings/buildings_1092-12.glb"},
]

## Schaffhausen/Schmerlat tiles
## Buildings were converted with origin E 2681968, N 1282869 (Schmerlat Airfield)
## Terrain center is also E 2681968, N 1282869 - no offset needed
const BUILDING_ORIGIN_OFFSET: Vector3 = Vector3(0, 0, 0)
const BUILDING_TILES_SCHAFFHAUSEN: Array[Dictionary] = [
	{"id": "1031-14", "center": Vector3(2903, 0, 2631), "path": "res://Assets/buildings_schaffhausen/buildings_1031-14.glb"},
	{"id": "1031-23", "center": Vector3(-1451, 0, 2597), "path": "res://Assets/buildings_schaffhausen/buildings_1031-23.glb"},
	{"id": "1031-24", "center": Vector3(-5892, 0, 2628), "path": "res://Assets/buildings_schaffhausen/buildings_1031-24.glb"},
	{"id": "1031-32", "center": Vector3(2914, 0, -363), "path": "res://Assets/buildings_schaffhausen/buildings_1031-32.glb"},
	{"id": "1031-34", "center": Vector3(2934, 0, -3347), "path": "res://Assets/buildings_schaffhausen/buildings_1031-34.glb"},
	{"id": "1031-41", "center": Vector3(-1517, 0, -363), "path": "res://Assets/buildings_schaffhausen/buildings_1031-41.glb"},
	{"id": "1031-42", "center": Vector3(-5844, 0, -361), "path": "res://Assets/buildings_schaffhausen/buildings_1031-42.glb"},
	{"id": "1031-43", "center": Vector3(-1331, 0, -3078), "path": "res://Assets/buildings_schaffhausen/buildings_1031-43.glb"},
	{"id": "1031-44", "center": Vector3(-5884, 0, -3368), "path": "res://Assets/buildings_schaffhausen/buildings_1031-44.glb"},
]

## Select which tile set to use based on terrain location
## Change this to switch between areas
enum TerrainArea { ZURICH, SCHAFFHAUSEN }
@export var terrain_area: TerrainArea = TerrainArea.SCHAFFHAUSEN

## Get the active building tiles based on terrain area
func _get_building_tiles() -> Array[Dictionary]:
	match terrain_area:
		TerrainArea.ZURICH:
			return BUILDING_TILES_ZURICH
		TerrainArea.SCHAFFHAUSEN:
			return BUILDING_TILES_SCHAFFHAUSEN
	return []

# State tracking
var _loaded_tiles: Dictionary = {}  # tile_id -> Node3D instance
var _loading_tiles: Dictionary = {}  # tile_id -> ResourceLoader ticket
var _pending_loads: Array[String] = []  # Queue of tile IDs to load

# Statistics
var tiles_loaded: int = 0
var tiles_in_memory: int = 0


func _ready() -> void:
	# Try to find aircraft node if not set
	if aircraft_node == null:
		aircraft_node = get_parent().get_node_or_null("AC")
		if aircraft_node == null:
			push_warning("BuildingLoader: No aircraft node set. Set aircraft_node export variable.")
	
	# Pre-calculate actual tile centers from the GLB bounds
	_calculate_tile_centers()


func _process(_delta: float) -> void:
	if not enabled or aircraft_node == null:
		return
	
	var aircraft_pos = aircraft_node.global_position
	
	# Check which tiles should be loaded/unloaded
	_update_tile_loading(aircraft_pos)
	
	# Process pending loads (limited per frame)
	_process_pending_loads()
	
	# Check async loading progress
	_check_loading_progress()


func _update_tile_loading(aircraft_pos: Vector3) -> void:
	var load_dist_sq = load_distance * load_distance
	var unload_dist_sq = unload_distance * unload_distance
	
	for tile_info in _get_building_tiles():
		var tile_id: String = tile_info["id"]
		var tile_center: Vector3 = tile_info["center"]
		
		# Calculate horizontal distance (ignore Y/altitude)
		var delta = aircraft_pos - tile_center
		delta.y = 0
		var dist_sq = delta.length_squared()
		
		var is_loaded = _loaded_tiles.has(tile_id)
		var is_loading = _loading_tiles.has(tile_id) or tile_id in _pending_loads
		
		if dist_sq < load_dist_sq:
			# Should be loaded
			if not is_loaded and not is_loading:
				_queue_tile_load(tile_id, tile_info["path"])
		elif dist_sq > unload_dist_sq:
			# Should be unloaded
			if is_loaded:
				_unload_tile(tile_id)
			elif is_loading:
				_cancel_tile_load(tile_id)


func _queue_tile_load(tile_id: String, path: String) -> void:
	if tile_id in _pending_loads:
		return
	
	_pending_loads.append(tile_id)
	if debug_mode:
		print("BuildingLoader: Queued tile %s for loading" % tile_id)


func _process_pending_loads() -> void:
	var loads_this_frame = 0
	
	while _pending_loads.size() > 0 and loads_this_frame < max_loads_per_frame:
		var tile_id = _pending_loads.pop_front()
		
		# Find tile info
		var tile_info = null
		for t in _get_building_tiles():
			if t["id"] == tile_id:
				tile_info = t
				break
		
		if tile_info == null:
			continue
		
		# Start async loading
		var path = tile_info["path"]
		if ResourceLoader.exists(path):
			ResourceLoader.load_threaded_request(path)
			_loading_tiles[tile_id] = path
			loads_this_frame += 1
			if debug_mode:
				print("BuildingLoader: Started loading tile %s" % tile_id)
		else:
			push_warning("BuildingLoader: GLB file not found: %s" % path)


func _check_loading_progress() -> void:
	var completed: Array[String] = []
	
	for tile_id in _loading_tiles.keys():
		var path = _loading_tiles[tile_id]
		var status = ResourceLoader.load_threaded_get_status(path)
		
		match status:
			ResourceLoader.THREAD_LOAD_LOADED:
				var resource = ResourceLoader.load_threaded_get(path)
				_instantiate_tile(tile_id, resource)
				completed.append(tile_id)
			ResourceLoader.THREAD_LOAD_FAILED:
				push_error("BuildingLoader: Failed to load tile %s" % tile_id)
				completed.append(tile_id)
			ResourceLoader.THREAD_LOAD_INVALID_RESOURCE:
				push_error("BuildingLoader: Invalid resource for tile %s" % tile_id)
				completed.append(tile_id)
	
	for tile_id in completed:
		_loading_tiles.erase(tile_id)


func _instantiate_tile(tile_id: String, resource: Resource) -> void:
	if resource == null or not resource is PackedScene:
		push_error("BuildingLoader: Resource is not a PackedScene for tile %s" % tile_id)
		return
	
	var instance = (resource as PackedScene).instantiate()
	instance.name = "Buildings_" + tile_id
	
	# Apply building offset (buildings already have correct Swiss ASL elevations)
	# Add terrain center offset if buildings were converted with different origin
	var offset = building_offset
	if terrain_area == TerrainArea.SCHAFFHAUSEN:
		offset += BUILDING_ORIGIN_OFFSET
	instance.position = offset
	
	# Apply building colors
	_apply_building_materials(instance)
	
	add_child(instance)
	_loaded_tiles[tile_id] = instance
	tiles_loaded += 1
	tiles_in_memory += 1
	
	if debug_mode:
		print("BuildingLoader: Loaded tile %s (total: %d tiles)" % [tile_id, tiles_in_memory])


func _unload_tile(tile_id: String) -> void:
	if not _loaded_tiles.has(tile_id):
		return
	
	var instance = _loaded_tiles[tile_id]
	if instance != null and is_instance_valid(instance):
		instance.queue_free()
	
	_loaded_tiles.erase(tile_id)
	tiles_in_memory -= 1
	
	if debug_mode:
		print("BuildingLoader: Unloaded tile %s (remaining: %d tiles)" % [tile_id, tiles_in_memory])


func _cancel_tile_load(tile_id: String) -> void:
	_pending_loads.erase(tile_id)
	_loading_tiles.erase(tile_id)


func _calculate_tile_centers() -> void:
	# This function can be used to recalculate tile centers from actual GLB bounds
	# For now, we use approximate centers based on Swiss tile grid
	# Each tile is roughly 3km x 3km
	pass



## Force load all tiles within a given distance
func force_load_nearby(distance: float) -> void:
	if aircraft_node == null:
		return
	
	var aircraft_pos = aircraft_node.global_position
	var dist_sq = distance * distance
	
	for tile_info in _get_building_tiles():
		var tile_id = tile_info["id"]
		var tile_center = tile_info["center"]
		
		var delta = aircraft_pos - tile_center
		delta.y = 0
		
		if delta.length_squared() < dist_sq:
			if not _loaded_tiles.has(tile_id) and not _loading_tiles.has(tile_id):
				_queue_tile_load(tile_id, tile_info["path"])


## Unload all tiles
func unload_all() -> void:
	for tile_id in _loaded_tiles.keys():
		_unload_tile(tile_id)
	_pending_loads.clear()
	_loading_tiles.clear()


## Get statistics
func get_stats() -> Dictionary:
	return {
		"tiles_loaded": tiles_loaded,
		"tiles_in_memory": tiles_in_memory,
		"tiles_loading": _loading_tiles.size(),
		"tiles_pending": _pending_loads.size(),
		"total_available": _get_building_tiles().size()
	}


## Apply building materials (roof and wall colors) to all meshes in the instance
func _apply_building_materials(node: Node) -> void:
	# Create shared materials (reused across all meshes)
	var roof_material = StandardMaterial3D.new()
	roof_material.albedo_color = roof_color
	roof_material.roughness = 0.8
	roof_material.metallic = 0.0
	
	var wall_material = StandardMaterial3D.new()
	wall_material.albedo_color = wall_color
	wall_material.roughness = 0.9
	wall_material.metallic = 0.0
	
	# Apply materials to all MeshInstance3D children
	_apply_materials_recursive(node, roof_material, wall_material)


func _apply_materials_recursive(node: Node, roof_mat: Material, wall_mat: Material) -> void:
	if node is MeshInstance3D:
		var mesh_instance = node as MeshInstance3D
		var mesh = mesh_instance.mesh
		if mesh:
			# Determine if this is a roof or wall based on normal direction
			# Roofs tend to have upward-facing normals, walls have horizontal normals
			# Since our buildings are simple geometry, we'll use a heuristic based on name or position
			# For now, we'll apply a mixed material that blends both colors based on vertex normals
			
			# Simple approach: create a shader material that colors based on normal
			var shader_material = _create_building_shader_material()
			mesh_instance.material_override = shader_material
	
	# Recurse into children
	for child in node.get_children():
		_apply_materials_recursive(child, roof_mat, wall_mat)


func _create_building_shader_material() -> ShaderMaterial:
	var shader = Shader.new()
	shader.code = """
shader_type spatial;

uniform vec4 roof_color : source_color = vec4(0.72, 0.38, 0.28, 1.0);
uniform vec4 wall_color : source_color = vec4(0.92, 0.87, 0.75, 1.0);
uniform float roof_threshold : hint_range(0.0, 1.0) = 0.5;

void fragment() {
	// Use world normal to determine roof vs wall
	// Roof faces have normals pointing up (positive Y in world space)
	vec3 world_normal = (INV_VIEW_MATRIX * vec4(NORMAL, 0.0)).xyz;
	float up_factor = max(0.0, world_normal.y);
	
	// Smoothly blend between wall and roof color based on normal
	float roof_blend = smoothstep(roof_threshold - 0.1, roof_threshold + 0.1, up_factor);
	
	ALBEDO = mix(wall_color.rgb, roof_color.rgb, roof_blend);
	ROUGHNESS = 0.85;
	METALLIC = 0.0;
}
"""
	
	var material = ShaderMaterial.new()
	material.shader = shader
	material.set_shader_parameter("roof_color", roof_color)
	material.set_shader_parameter("wall_color", wall_color)
	material.set_shader_parameter("roof_threshold", 0.5)
	
	return material
