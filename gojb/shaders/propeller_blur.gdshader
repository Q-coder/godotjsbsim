shader_type spatial;
render_mode blend_mix, depth_draw_opaque, cull_disabled, unshaded;

// Propeller blur disc shader
// Creates a realistic motion blur effect for spinning propellers

uniform float rpm : hint_range(0.0, 3000.0) = 1000.0;
uniform int blade_count : hint_range(2, 6) = 2;
uniform float hub_radius : hint_range(0.0, 0.5) = 0.08;
uniform float blade_width : hint_range(0.01, 0.3) = 0.12;
uniform vec4 blade_color : source_color = vec4(0.08, 0.08, 0.08, 1.0);
uniform vec4 blur_color : source_color = vec4(0.12, 0.12, 0.15, 0.7);
uniform float blur_intensity : hint_range(0.0, 1.0) = 0.8;

void fragment() {
	// Convert UV to centered coordinates (-1 to 1)
	vec2 uv_centered = UV * 2.0 - 1.0;
	
	// Distance from center
	float dist = length(uv_centered);
	
	// Discard outside the disc (make it circular)
	if (dist > 1.0) {
		discard;
	}
	
	// Hub area (solid dark center)
	if (dist < hub_radius) {
		ALBEDO = blade_color.rgb * 0.8;
		ALPHA = 0.9;
		return;
	}
	
	// Calculate angle for blade pattern
	float angle = atan(uv_centered.y, uv_centered.x);
	
	// Blade pattern - creates blade_count blades
	float blade_angle = 6.28318530718 / float(blade_count); // 2*PI / blade_count
	float blade_pattern = abs(mod(angle, blade_angle) - blade_angle * 0.5);
	
	// RPM factor: 0 at low RPM, 1 at high RPM
	float rpm_factor = clamp(rpm / 1500.0, 0.0, 1.0);
	
	// Blade visibility - sharper at low RPM, blurred at high RPM
	float blade_sharpness = mix(blade_width * 0.4, blade_angle * 0.5, rpm_factor);
	float blade_visibility = smoothstep(blade_sharpness, blade_sharpness * 0.2, blade_pattern);
	
	// Radial fade at the tips
	float tip_fade = smoothstep(1.0, 0.7, dist);
	float inner_fade = smoothstep(hub_radius, hub_radius + 0.15, dist);
	
	// Final color and alpha
	vec3 final_color;
	float final_alpha;
	
	// Blend from distinct blades to uniform blur based on RPM
	float blur_amount = smoothstep(0.3, 0.8, rpm_factor);
	
	// Color: dark blades transitioning to slightly lighter blur
	final_color = mix(blade_color.rgb, blur_color.rgb, blur_amount * 0.5);
	
	// Alpha: blade pattern fading to uniform blur disc
	float blade_alpha = blade_visibility * tip_fade * inner_fade * 0.85;
	float blur_alpha = blur_intensity * tip_fade * inner_fade * 0.6;
	final_alpha = mix(blade_alpha, blur_alpha, blur_amount);
	
	// Ensure minimum visibility
	final_alpha = max(final_alpha, 0.15 * tip_fade * inner_fade);
	
	ALBEDO = final_color;
	ALPHA = final_alpha;
}
