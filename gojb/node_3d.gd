extends Node3D

# Assuming your JSBGodot node is a child of the current node
var jsb_node: Node
var active_camera: Camera3D
var camera1: Camera3D
var camera2: Camera3D
var camera1_base_transform: Transform3D  # Store original camera transform

# Pilot head look angles (in degrees) - for cockpit camera
var head_yaw: float = 0.0    # Left/Right
var head_pitch: float = 0.0  # Up/Down
const HEAD_YAW_MAX: float = 120.0   # Max look left/right
const HEAD_PITCH_MAX: float = 60.0  # Max look up/down
const HEAD_LOOK_SPEED: float = 90.0 # Degrees per second

# Spot camera orbit angles (in degrees) - for external camera
var orbit_yaw: float = 135.0    # Horizontal angle around aircraft (start behind-left)
var orbit_pitch: float = 15.0   # Vertical angle (elevation)
var orbit_distance: float = 15.0  # Distance from aircraft
const ORBIT_PITCH_MIN: float = -10.0   # Min elevation (slightly below)
const ORBIT_PITCH_MAX: float = 80.0    # Max elevation (almost top-down)
const ORBIT_SPEED: float = 90.0        # Degrees per second
const ORBIT_ZOOM_SPEED: float = 10.0   # Units per second
const ORBIT_DISTANCE_MIN: float = 5.0  # Closest zoom
const ORBIT_DISTANCE_MAX: float = 50.0 # Furthest zoom

# Control surface nodes
var rudder_node: Node3D
var rudder_base_transform: Transform3D  # Store original transform
const RUDDER_MAX_ANGLE: float = 16.0  # Realistic rudder deflection
# Disable pivot offset - not needed now that we use the MeshInstance3D
var rudder_pivot_offset: Vector3 = Vector3(0, 0, 0)

# Propeller
var propeller_node: Node3D
var propeller_blur_disc: MeshInstance3D  # Blur disc mesh
var propeller_blur_material: Material  # Blur material (can be Standard or Shader)
var propeller_rotation: float = 0.0  # Current rotation angle
const PROPELLER_MAX_RPM: float = 2700.0  # Max RPM for reference
const PROPELLER_BLUR_START_RPM: float = 900.0  # RPM where blur disc appears (above idle)
const PROPELLER_HIDE_BLADES_RPM: float = 1200.0  # RPM where real blades hidden
const PROPELLER_DISC_RADIUS: float = 0.95  # Radius of blur disc in meters

# Ailerons
var left_aileron_node: Node3D
var right_aileron_node: Node3D
var left_aileron_base_transform: Transform3D
var right_aileron_base_transform: Transform3D
const AILERON_MAX_ANGLE: float = 20.0  # Max aileron deflection in degrees

# Elevators
var left_elevator_node: Node3D
var right_elevator_node: Node3D
var left_elevator_base_transform: Transform3D
var right_elevator_base_transform: Transform3D
const ELEVATOR_MAX_ANGLE: float = 25.0  # Max elevator deflection in degrees

# Flaps
var left_flap_node: Node3D
var right_flap_node: Node3D
var left_flap_base_transform: Transform3D
var right_flap_base_transform: Transform3D
const FLAP_MAX_ANGLE: float = 30.0  # C172p max flap deflection (0, 10, 20, 30 degrees)

# Front wheel (nose gear steering)
var front_wheel_node: Node3D
var front_wheel_base_transform: Transform3D
const FRONT_WHEEL_MAX_ANGLE: float = 30.0  # Max steering angle

# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	print("Node3D _ready() starting...")
	# Get JSBGodot node after we're in the scene tree
	jsb_node = $AC/JSBGodot
	print("JSBGodot node: ", jsb_node)
	
	# Access the AC node and its cameras
	var ac_node = $AC
	print("AC node: ", ac_node)
	camera1 = ac_node.get_node("Camera1")
	camera2 = ac_node.get_node("Camera2")
	active_camera = camera1
	camera1_base_transform = camera1.transform  # Store base transform
	print("Camera1: ", camera1)
	print("Camera2: ", camera2)
	
	# Set Camera1 as the active one initially
	active_camera.current = true
	camera2.current = false
	
	# Get control surface nodes for animation - using the MeshInstance3D nodes
	rudder_node = get_node("AC/Node3D2/C172P_1/Sketchfab_model/Cessna172_fbx/RootNode/75f3edaeef374a9f89e7b5ef606a0759_fbx/RootNode_001/Cessna-172/Cessna_Exterior/Cessna_Exterior_Body_MAT_0_008/Cessna_Exterior_Body_MAT_0_008_Rudder")
	if rudder_node:
		print("Rudder node found: ", rudder_node)
		rudder_base_transform = rudder_node.transform  # Store original transform
	else:
		print("WARNING: Rudder node not found!")
	
	# Get propeller node - using the MeshInstance3D node
	propeller_node = get_node("AC/Node3D2/C172P_1/Sketchfab_model/Cessna172_fbx/RootNode/75f3edaeef374a9f89e7b5ef606a0759_fbx/RootNode_001/Cessna-172/Cessna_Exterior/Cessna_Exterior_Body_MAT_0_001/Cessna_Exterior_Body_MAT_0_001_Body_MAT_0")
	if propeller_node:
		print("Propeller node found: ", propeller_node)
		# Create the propeller blur disc
		_create_propeller_blur_disc()
	else:
		print("WARNING: Propeller node not found!")
	
	# Get aileron nodes - using the MeshInstance3D for animation
	left_aileron_node = get_node("AC/Node3D2/C172P_1/Sketchfab_model/Cessna172_fbx/RootNode/75f3edaeef374a9f89e7b5ef606a0759_fbx/RootNode_001/Cessna-172/Cessna_Exterior/Cessna_Exterior_Body_MAT_0_014/Cessna_Exterior_Body_MAT_0_014_Aileron_Left")
	if left_aileron_node:
		print("Left aileron node found: ", left_aileron_node)
		left_aileron_base_transform = left_aileron_node.transform
	else:
		print("WARNING: Left aileron node not found!")
	
	right_aileron_node = get_node("AC/Node3D2/C172P_1/Sketchfab_model/Cessna172_fbx/RootNode/75f3edaeef374a9f89e7b5ef606a0759_fbx/RootNode_001/Cessna-172/Cessna_Exterior/Cessna_Exterior_Body_MAT_0_015/Cessna_Exterior_Body_MAT_0_015_Aileron_Right")
	if right_aileron_node:
		print("Right aileron node found: ", right_aileron_node)
		right_aileron_base_transform = right_aileron_node.transform
	else:
		print("WARNING: Right aileron node not found!")
	
	# Get elevator nodes
	left_elevator_node = get_node("AC/Node3D2/C172P_1/Sketchfab_model/Cessna172_fbx/RootNode/75f3edaeef374a9f89e7b5ef606a0759_fbx/RootNode_001/Cessna-172/Cessna_Exterior/Cessna_Exterior_Body_MAT_0_013/Cessna_Exterior_Body_MAT_0_013_Elevator_Left")
	if left_elevator_node:
		print("Left elevator node found: ", left_elevator_node)
		left_elevator_base_transform = left_elevator_node.transform
	else:
		print("WARNING: Left elevator node not found!")
	
	right_elevator_node = get_node("AC/Node3D2/C172P_1/Sketchfab_model/Cessna172_fbx/RootNode/75f3edaeef374a9f89e7b5ef606a0759_fbx/RootNode_001/Cessna-172/Cessna_Exterior/Cessna_Exterior_Body_MAT_0_012/Cessna_Exterior_Body_MAT_0_012_Elevator_Right")
	if right_elevator_node:
		print("Right elevator node found: ", right_elevator_node)
		right_elevator_base_transform = right_elevator_node.transform
	else:
		print("WARNING: Right elevator node not found!")
	
	# Get flap nodes
	left_flap_node = get_node("AC/Node3D2/C172P_1/Sketchfab_model/Cessna172_fbx/RootNode/75f3edaeef374a9f89e7b5ef606a0759_fbx/RootNode_001/Cessna-172/Cessna_Exterior/Cessna_Exterior_Body_MAT_0_016/Cessna_Exterior_Body_MAT_0_016_Flaps_Left")
	if left_flap_node:
		print("Left flap node found: ", left_flap_node)
		left_flap_base_transform = left_flap_node.transform
	else:
		print("WARNING: Left flap node not found!")
	
	right_flap_node = get_node("AC/Node3D2/C172P_1/Sketchfab_model/Cessna172_fbx/RootNode/75f3edaeef374a9f89e7b5ef606a0759_fbx/RootNode_001/Cessna-172/Cessna_Exterior/Cessna_Exterior_Body_MAT_0_017/Cessna_Exterior_Body_MAT_0_017_Flaps_Right")
	if right_flap_node:
		print("Right flap node found: ", right_flap_node)
		right_flap_base_transform = right_flap_node.transform
	else:
		print("WARNING: Right flap node not found!")
	
	# Get front wheel node for nose gear steering
	# The VehicleWheel3D node is: Cessna_Exterior_Body_MAT_0_021_Body_Front
	front_wheel_node = get_node_or_null("AC/Node3D2/C172P_1/Sketchfab_model/Cessna172_fbx/RootNode/75f3edaeef374a9f89e7b5ef606a0759_fbx/RootNode_001/Cessna-172/Cessna_Exterior/Cessna_Exterior_Body_MAT_0_021/Cessna_Exterior_Body_MAT_0_021_Body_Front")
	if front_wheel_node:
		print("Front wheel node found: ", front_wheel_node)
		front_wheel_base_transform = front_wheel_node.transform
	else:
		print("WARNING: Front wheel node not found!")
	
	# Print the aircraft model hierarchy to find control surfaces
	print("=== Aircraft Node Hierarchy ===")
	print_node_tree($AC, 0)
	print("=== End Hierarchy ===")
	
	print("Node3D _ready() complete!")


# Create the propeller blur disc mesh
func _create_propeller_blur_disc() -> void:
	# Use StandardMaterial3D - shaders have transparency issues
	var blur_mat = StandardMaterial3D.new()
	blur_mat.albedo_color = Color(0.15, 0.15, 0.18, 0.1)  # Dark gray, more transparent
	blur_mat.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
	blur_mat.cull_mode = BaseMaterial3D.CULL_DISABLED  # Visible from both sides
	blur_mat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED  # No lighting
	propeller_blur_material = blur_mat
	
	# Get the propeller mesh's bounding box to determine disc size
	if propeller_node is MeshInstance3D:
		var mesh_inst = propeller_node as MeshInstance3D
		var aabb = mesh_inst.get_aabb()
		var mesh_center = aabb.position + aabb.size / 2.0
		
		# Propeller diameter is the Y size (blades extend along Y axis)
		var prop_diameter = aabb.size.y
		var prop_radius = prop_diameter / 2.0
		print("Propeller diameter: ", prop_diameter, " cm")
		
		# Create a circular disc using CylinderMesh (very thin cylinder = disc)
		var disc_mesh = CylinderMesh.new()
		disc_mesh.top_radius = prop_radius
		disc_mesh.bottom_radius = prop_radius
		disc_mesh.height = 0.5  # Very thin
		disc_mesh.radial_segments = 32  # Smooth circle
		disc_mesh.rings = 1
		
		# Create MeshInstance3D for the blur disc
		propeller_blur_disc = MeshInstance3D.new()
		propeller_blur_disc.mesh = disc_mesh
		propeller_blur_disc.material_override = propeller_blur_material
		propeller_blur_disc.visible = false  # Start hidden
		
		# Add as sibling to propeller's parent
		var prop_parent = propeller_node.get_parent()
		if prop_parent:
			prop_parent.add_child(propeller_blur_disc)
			# Position at the center of the propeller mesh geometry
			propeller_blur_disc.position = mesh_center
			# CylinderMesh stands upright (Y axis), rotate to face forward (Z axis)
			propeller_blur_disc.rotation_degrees = Vector3(90, 0, 0)
			print("Blur disc created with diameter: ", prop_diameter)
	else:
		print("WARNING: Propeller is not a MeshInstance3D!")


# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta: float) -> void:
	if Input.is_action_just_pressed("flip_camera"):
		flip_camera()
	
	# Handle hat switch / D-pad for pilot head look or orbit camera
	if active_camera == camera1:
		handle_head_look(delta)
	else:
		handle_orbit_camera(delta)
	
	# Animate control surfaces
	animate_control_surfaces()
		
	if jsb_node:
		var airspeed = jsb_node.get_airspeed_knots()
		var altitude_ft = jsb_node.get_altitude_ft()
		var vertical_spd = jsb_node.get_vertical_speed_fpm()
		var throttle = jsb_node.get_input_throttle()
		var throttle_percent = throttle * 100.0
		var heading = jsb_node.get_heading()

		$Control/Label.text = "Airspeed: " + str(round(airspeed)) + " knots"
		$Control/Label2.text = "Altitude: " + str(round(altitude_ft)) + " feet"
		$Control/Label3.text = "Verical Speed: " + str(round(vertical_spd)) + " ft/m"
		$Control/Label4.text = "Throttle: " +str(round(throttle_percent)) + " %"
		$Control/Label5.text = "Heading: " +str(round(heading))
	else:
		$Label.text = "JSBGodot node not found."

# Function to toggle between the two cameras in the AC node
func flip_camera():
	var ac_node = $AC
	if active_camera == camera1:
		active_camera = camera2
	else:
		active_camera = camera1
		# Reset head position when switching back to cockpit view
		head_yaw = 0.0
		head_pitch = 0.0

	# Set the current camera
	camera1.current = (active_camera == camera1)
	camera2.current = (active_camera == camera2)

# Handle pilot head look using hat switch or D-pad
func handle_head_look(delta: float) -> void:
	# Only apply head look when using cockpit camera (Camera1)
	if active_camera != camera1:
		return
	
	# Get hat switch input (typically axes 6 and 7 on many controllers)
	# Or use D-pad which maps to JOY_BUTTON_DPAD_*
	var look_x: float = 0.0
	var look_y: float = 0.0
	
	# Try hat switch axes first (JOY_AXIS 6 = hat X, JOY_AXIS 7 = hat Y on some controllers)
	look_x = Input.get_joy_axis(0, JOY_AXIS_MAX)  # Some controllers use higher axes
	look_y = Input.get_joy_axis(0, JOY_AXIS_MAX)
	
	# If no hat axis input, check D-pad buttons
	if abs(look_x) < 0.1 and abs(look_y) < 0.1:
		if Input.is_joy_button_pressed(0, JOY_BUTTON_DPAD_LEFT):
			look_x = -1.0
		elif Input.is_joy_button_pressed(0, JOY_BUTTON_DPAD_RIGHT):
			look_x = 1.0
		if Input.is_joy_button_pressed(0, JOY_BUTTON_DPAD_UP):
			look_y = -1.0  # Up on D-pad = look up (negative pitch)
		elif Input.is_joy_button_pressed(0, JOY_BUTTON_DPAD_DOWN):
			look_y = 1.0   # Down on D-pad = look down (positive pitch)
	
	# Also support keyboard arrows for testing
	if Input.is_key_pressed(KEY_LEFT):
		look_x = -1.0
	elif Input.is_key_pressed(KEY_RIGHT):
		look_x = 1.0
	if Input.is_key_pressed(KEY_UP):
		look_y = -1.0
	elif Input.is_key_pressed(KEY_DOWN):
		look_y = 1.0
	
	# Update head angles
	head_yaw += look_x * HEAD_LOOK_SPEED * delta
	head_pitch += look_y * HEAD_LOOK_SPEED * delta
	
	# Clamp to limits
	head_yaw = clamp(head_yaw, -HEAD_YAW_MAX, HEAD_YAW_MAX)
	head_pitch = clamp(head_pitch, -HEAD_PITCH_MAX, HEAD_PITCH_MAX)
	
	# Apply rotation to camera (relative to base transform)
	# Start from base transform, then apply head rotation
	camera1.transform = camera1_base_transform
	camera1.rotate_y(deg_to_rad(-head_yaw))  # Yaw (left/right)
	camera1.rotate_object_local(Vector3.RIGHT, deg_to_rad(-head_pitch))  # Pitch (up/down)

# Handle external/spot camera orbit around aircraft
func handle_orbit_camera(delta: float) -> void:
	# Get input from hat switch, D-pad, or keyboard
	var look_x: float = 0.0
	var look_y: float = 0.0
	
	# Check D-pad buttons
	if Input.is_joy_button_pressed(0, JOY_BUTTON_DPAD_LEFT):
		look_x = -1.0
	elif Input.is_joy_button_pressed(0, JOY_BUTTON_DPAD_RIGHT):
		look_x = 1.0
	if Input.is_joy_button_pressed(0, JOY_BUTTON_DPAD_UP):
		look_y = 1.0   # Up = increase elevation
	elif Input.is_joy_button_pressed(0, JOY_BUTTON_DPAD_DOWN):
		look_y = -1.0  # Down = decrease elevation
	
	# Also support keyboard arrows for testing
	if Input.is_key_pressed(KEY_LEFT):
		look_x = -1.0
	elif Input.is_key_pressed(KEY_RIGHT):
		look_x = 1.0
	if Input.is_key_pressed(KEY_UP):
		look_y = 1.0
	elif Input.is_key_pressed(KEY_DOWN):
		look_y = -1.0
	
	# Update orbit angles
	orbit_yaw += look_x * ORBIT_SPEED * delta
	orbit_pitch += look_y * ORBIT_SPEED * delta
	
	# Wrap yaw (allow full 360 rotation)
	if orbit_yaw > 360.0:
		orbit_yaw -= 360.0
	elif orbit_yaw < 0.0:
		orbit_yaw += 360.0
	
	# Clamp pitch to limits
	orbit_pitch = clamp(orbit_pitch, ORBIT_PITCH_MIN, ORBIT_PITCH_MAX)
	
	# Handle zoom - L1/R1 (shoulder buttons) or Page Up/Page Down
	var zoom_in = 0.0
	var zoom_out = 0.0
	
	# Gamepad shoulder buttons (L1 = button 4, R1 = button 5)
	if Input.is_joy_button_pressed(0, JOY_BUTTON_LEFT_SHOULDER):
		zoom_in = 1.0
	if Input.is_joy_button_pressed(0, JOY_BUTTON_RIGHT_SHOULDER):
		zoom_out = 1.0
	
	# Keyboard Page Up/Down
	if Input.is_key_pressed(KEY_PAGEUP):
		zoom_in = 1.0
	if Input.is_key_pressed(KEY_PAGEDOWN):
		zoom_out = 1.0
	
	# Update distance
	orbit_distance -= zoom_in * ORBIT_ZOOM_SPEED * delta
	orbit_distance += zoom_out * ORBIT_ZOOM_SPEED * delta
	orbit_distance = clamp(orbit_distance, ORBIT_DISTANCE_MIN, ORBIT_DISTANCE_MAX)
	
	# Calculate camera offset in world space (not affected by aircraft rotation)
	var yaw_rad = deg_to_rad(orbit_yaw)
	var pitch_rad = deg_to_rad(orbit_pitch)
	
	# Spherical to Cartesian conversion (world-aligned offset)
	var offset_x = orbit_distance * cos(pitch_rad) * sin(yaw_rad)
	var offset_y = orbit_distance * sin(pitch_rad)
	var offset_z = orbit_distance * cos(pitch_rad) * cos(yaw_rad)
	var world_offset = Vector3(offset_x, offset_y, offset_z)
	
	# Get the AC node (parent of camera2) and its global position
	var ac_node = $AC
	var aircraft_global_pos = ac_node.global_position
	
	# Set camera to world position (aircraft position + world-aligned offset)
	camera2.global_position = aircraft_global_pos + world_offset
	
	# Make camera look at the aircraft center in world space
	camera2.look_at(aircraft_global_pos, Vector3.UP)

# Helper to print node tree
func print_node_tree(node: Node, depth: int) -> void:
	var indent = ""
	for i in range(depth):
		indent += "  "
	print(indent + node.name + " [" + node.get_class() + "]")
	for child in node.get_children():
		print_node_tree(child, depth + 1)

# Adjust rudder pivot offset in real-time for experimentation
# Keys: U/J = X axis, I/K = Y axis, O/L = Z axis
func handle_pivot_adjustment() -> void:
	var step = 0.1
	var changed = false
	
	if Input.is_key_pressed(KEY_U):
		rudder_pivot_offset.x += step * get_process_delta_time() * 5
		changed = true
	elif Input.is_key_pressed(KEY_J):
		rudder_pivot_offset.x -= step * get_process_delta_time() * 5
		changed = true
	
	if Input.is_key_pressed(KEY_I):
		rudder_pivot_offset.y += step * get_process_delta_time() * 5
		changed = true
	elif Input.is_key_pressed(KEY_K):
		rudder_pivot_offset.y -= step * get_process_delta_time() * 5
		changed = true
	
	if Input.is_key_pressed(KEY_O):
		rudder_pivot_offset.z += step * get_process_delta_time() * 5
		changed = true
	elif Input.is_key_pressed(KEY_L):
		rudder_pivot_offset.z -= step * get_process_delta_time() * 5
		changed = true
	
	if changed:
		print("Pivot offset: ", rudder_pivot_offset)

# Animate control surfaces based on JSBSim inputs
func animate_control_surfaces() -> void:
	if not jsb_node:
		return
	
	# Animate rudder - rotate around LOCAL Y axis (hinge line as set in Blender)
	if rudder_node:
		var rudder_input = jsb_node.get_input_rudder()  # -1 to 1
		var rudder_angle = -rudder_input * RUDDER_MAX_ANGLE
		
		# Reset to base transform first, then apply LOCAL rotation around Y axis
		rudder_node.transform = rudder_base_transform
		rudder_node.rotate_object_local(Vector3.UP, deg_to_rad(rudder_angle))
	
	# Animate front wheel (nose gear steering) - opposite to rudder
	if front_wheel_node:
		var rudder_input = jsb_node.get_input_rudder()  # -1 to 1
		# Opposite direction: positive rudder (right) = wheel turns left
		var wheel_angle = rudder_input * FRONT_WHEEL_MAX_ANGLE
		
		# Reset to base transform first, then apply rotation
		front_wheel_node.transform = front_wheel_base_transform
		# Try rotating around local Y axis (vertical when wheel is upright)
		front_wheel_node.rotate_object_local(Vector3.UP, deg_to_rad(wheel_angle))
	
	# Animate propeller based on actual RPM from JSBSim
	if propeller_node:
		# Get actual propeller RPM from JSBSim physics
		var rpm = jsb_node.get_propeller_rpm()
		
		# Debug RPM every few seconds
		if Engine.get_frames_drawn() % 120 == 0:
			print("Propeller RPM: ", rpm, " blur_disc: ", propeller_blur_disc != null)
		
		# Convert RPM to degrees per second (RPM * 360 / 60 = RPM * 6)
		var degrees_per_second = rpm * 6.0
		
		# Update rotation (get_process_delta_time for smooth animation)
		propeller_rotation += degrees_per_second * get_process_delta_time()
		
		# Keep angle in 0-360 range
		propeller_rotation = fmod(propeller_rotation, 360.0)
		
		# Apply rotation around Z axis (forward axis of aircraft)
		propeller_node.rotation_degrees.z = propeller_rotation
		
		# Swap between real propeller blades and blur disc based on RPM
		if propeller_blur_disc:
			# Update shader RPM parameter if using shader material
			if propeller_blur_material and propeller_blur_material is ShaderMaterial:
				propeller_blur_material.set_shader_parameter("rpm", rpm)
			
			if rpm < PROPELLER_BLUR_START_RPM:
				# Low RPM - show real blades, hide blur disc
				propeller_node.visible = true
				propeller_blur_disc.visible = false
			elif rpm < PROPELLER_HIDE_BLADES_RPM:
				# Transition zone - show both with blur disc fading in
				propeller_node.visible = true
				propeller_blur_disc.visible = true
			else:
				# High RPM - hide real blades, show only blur disc
				propeller_node.visible = false
				propeller_blur_disc.visible = true
		else:
			# No blur disc available - always show real propeller
			propeller_node.visible = true
	
	# Animate ailerons - they move opposite to each other for roll control
	var aileron_input = jsb_node.get_input_aileron()  # -1 to 1
	# Debug aileron input
	if abs(aileron_input) > 0.01:
		print("Aileron input: ", aileron_input)
	
	if left_aileron_node:
		var left_aileron_angle = aileron_input * AILERON_MAX_ANGLE
		left_aileron_node.transform = left_aileron_base_transform
		# Rotate around X axis (along the wing span)
		left_aileron_node.rotation_degrees.x = left_aileron_base_transform.basis.get_euler().x * (180.0/PI) + left_aileron_angle
	
	if right_aileron_node:
		var right_aileron_angle = -aileron_input * AILERON_MAX_ANGLE  # Opposite direction from left aileron
		right_aileron_node.transform = right_aileron_base_transform
		right_aileron_node.rotation_degrees.x = right_aileron_base_transform.basis.get_euler().x * (180.0/PI) + right_aileron_angle
	
	# Animate elevators - both move in the same direction
	var elevator_input = jsb_node.get_input_elevator()  # -1 to 1
	if left_elevator_node:
		var elevator_angle = -elevator_input * ELEVATOR_MAX_ANGLE  # Inverted
		left_elevator_node.transform = left_elevator_base_transform
		left_elevator_node.rotation_degrees.x = left_elevator_base_transform.basis.get_euler().x * (180.0/PI) + elevator_angle
	
	if right_elevator_node:
		var elevator_angle = -elevator_input * ELEVATOR_MAX_ANGLE  # Inverted
		right_elevator_node.transform = right_elevator_base_transform
		right_elevator_node.rotation_degrees.x = right_elevator_base_transform.basis.get_euler().x * (180.0/PI) + elevator_angle
	
	# Animate flaps - both move in the same direction (down only, 0 to max)
	var flap_input = jsb_node.get_flaps()  # 0 to 1
	if left_flap_node:
		var flap_angle = -flap_input * FLAP_MAX_ANGLE  # Inverted
		left_flap_node.transform = left_flap_base_transform
		left_flap_node.rotation_degrees.x = left_flap_base_transform.basis.get_euler().x * (180.0/PI) + flap_angle
	
	if right_flap_node:
		var flap_angle = -flap_input * FLAP_MAX_ANGLE  # Inverted
		right_flap_node.transform = right_flap_base_transform
		right_flap_node.rotation_degrees.x = right_flap_base_transform.basis.get_euler().x * (180.0/PI) + flap_angle
