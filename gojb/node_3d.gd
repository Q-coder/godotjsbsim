extends Node3D

# Assuming your JSBGodot node is a child of the current node
@onready var jsb_node = get_node("AC/JSBGodot")  # Adjust the path as needed
var active_camera: Camera3D

# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	# Access the AC node and its cameras
	var ac_node = $AC
	active_camera = ac_node.get_node("Camera1")
	
	# Set Camera1 as the active one initially
	active_camera.current = true
	ac_node.get_node("Camera2").current = false


# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta: float) -> void:
	if Input.is_action_just_pressed("flip_camera"):
		flip_camera()
		
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
	if active_camera == ac_node.get_node("Camera1"):
		active_camera = ac_node.get_node("Camera2")
	else:
		active_camera = ac_node.get_node("Camera1")

	# Set the current camera
	ac_node.get_node("Camera1").current = (active_camera == ac_node.get_node("Camera1"))
	ac_node.get_node("Camera2").current = (active_camera == ac_node.get_node("Camera2"))
