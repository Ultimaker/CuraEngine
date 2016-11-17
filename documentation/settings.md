# Settings
## Command Line Settings
Key|Label|Description|Type|Settable
---|-----|-----------|----|--------
center_object|Center object|Whether to center the object on the middle of the build platform (0,0), instead of using the coordinate system in which the object was saved.|bool|Globally<br/>Extruder
machine_print_temp_wait|Machine print temp wait|Whether to wait for the nozzle temperature to be reached when preheating the nozzles at the start of the gcode.|bool|Globally<br/>Extruder
mesh_position_x|Mesh position x|Offset applied to the object in the x direction.|float|Globally<br/>Extruder
mesh_position_y|Mesh position y|Offset applied to the object in the y direction.|float|Globally<br/>Extruder
mesh_position_z|Mesh position z|Offset applied to the object in the z direction. With this you can perform what was used to call 'Object Sink'.|float|Globally<br/>Extruder
mesh_rotation_matrix|Mesh Rotation Matrix|Transformation matrix to be applied to the model when loading it from file.|string|Globally<br/>Extruder
prime_tower_dir_outward|Prime tower direction outward|Whether to start printing in the middle of the prime tower and end up at the perimeter, or the other way around. This is only used for certain types of prime tower.|bool|Globally<br/>Extruder

## Printer Settings
Key|Label|Description|Type|Settable
---|-----|-----------|----|--------
machine_start_gcode|Start GCode|Gcode commands to be executed at the very start - separated by \n.|str|Globally
machine_end_gcode|End GCode|Gcode commands to be executed at the very end - separated by \n.|str|Globally
material_bed_temp_wait|Wait for build plate heatup|Whether to insert a command to wait until the build plate temperature is reached at the start.|bool|Globally
material_print_temp_wait|Wait for nozzle heatup|Whether to wait until the nozzle temperature is reached at the start.|bool|Globally
material_print_temp_prepend|Include material temperatures|Whether to include nozzle temperature commands at the start of the gcode. When the start_gcode already contains nozzle temperature commands Cura frontend will automatically disable this setting.|bool|Globally
material_bed_temp_prepend|Include build plate temperature|Whether to include build plate temperature commands at the start of the gcode. When the start_gcode already contains build plate temperature commands Cura frontend will automatically disable this setting.|bool|Globally
machine_width|Machine width|The width (X-direction) of the printable area.|float|Globally
machine_depth|Machine depth|The depth (Y-direction) of the printable area.|float|Globally
machine_height|Machine height|The height (Z-direction) of the printable area.|float|Globally
machine_heated_bed|Has heated build plate|Whether the machine has a heated build plate present.|bool|Globally
machine_center_is_zero|Is center origin|Whether the X/Y coordinates of the zero position of the printer is at the center of the printable area.|bool|Globally
machine_extruder_count|Number extruders|Number of extruder trains. An extruder train is the combination of a feeder, bowden tube, and nozzle.|int|Globally
machine_nozzle_tip_outer_diameter|Outer nozzle diameter|The outer diameter of the tip of the nozzle.|float|Extruder
machine_nozzle_head_distance|Nozzle length|The height difference between the tip of the nozzle and the lowest part of the print head.|float|Globally
machine_nozzle_expansion_angle|Nozzle angle|The angle between the horizontal plane and the conical part right above the tip of the nozzle.|int|Globally
machine_heat_zone_length|Heat zone length|The distance from the tip of the nozzle in which heat from the nozzle is transfered to the filament.|float|Globally<br/>Extruder
machine_nozzle_heat_up_speed|Heat up speed|The speed (°C/s) by which the nozzle heats up averaged over the window of normal printing temperatures and the standby temperature.|float|Globally<br/>Extruder
machine_nozzle_cool_down_speed|Cool down speed|The speed (°C/s) by which the nozzle cools down averaged over the window of normal printing temperatures and the standby temperature.|float|Globally<br/>Extruder
machine_min_cool_heat_time_window|Minimal Time Standby Temperature|The minimal time an extruder has to be inactive before the nozzle is cooled. Only when an extruder is not used for longer than this time will it be allowed to cool down to the standby temperature.|float|Globally<br/>Extruder
machine_gcode_flavor|Gcode flavour|The type of gcode to be generated.|enum|Globally
machine_disallowed_areas|Disallowed areas|A list of polygons with areas the print head is not allowed to enter.|polygons|Globally
machine_head_polygon|Machine head polygon|A 2D silhouette of the print head (fan caps excluded).|polygon|Globally
machine_head_with_fans_polygon|Machine head & Fan polygon|A 2D silhouette of the print head (fan caps included).|polygon|Globally
gantry_height|Gantry height|The height difference between the tip of the nozzle and the gantry system (X and Y axes).|float|Globally
machine_nozzle_size|Nozzle Diameter|The inner diameter of the nozzle. Change this setting when using a non-standard nozzle size.|float|Globally<br/>Extruder
machine_use_extruder_offset<br/>_to_offset_coords|Offset With Extruder|Apply the extruder offset to the coordinate system.|bool|Globally
extruder_prime_pos_z|Extruder Prime Z Position|The Z coordinate of the position where the nozzle primes at the start of printing.|float|Globally<br/>Extruder
extruder_prime_pos_abs|Absolute Extruder Prime Position|Make the extruder prime position absolute rather than relative to the last-known location of the head.|bool|Globally<br/>Extruder
machine_max_feedrate_x|Maximum Speed X|The maximum speed for the motor of the X-direction.|float|Globally
machine_max_feedrate_y|Maximum Speed Y|The maximum speed for the motor of the Y-direction.|float|Globally
machine_max_feedrate_z|Maximum Speed Z|The maximum speed for the motor of the Z-direction.|float|Globally
machine_max_feedrate_e|Maximum Feedrate|The maximum speed of the filament.|float|Globally
machine_max_acceleration_x|Maximum Acceleration X|Maximum acceleration for the motor of the X-direction|float|Globally
machine_max_acceleration_y|Maximum Acceleration Y|Maximum acceleration for the motor of the Y-direction.|float|Globally
machine_max_acceleration_z|Maximum Acceleration Z|Maximum acceleration for the motor of the Z-direction.|float|Globally
machine_max_acceleration_e|Maximum Filament Acceleration|Maximum acceleration for the motor of the filament.|float|Globally
machine_acceleration|Default Acceleration|The default acceleration of print head movement.|float|Globally
machine_max_jerk_xy|Default X-Y Jerk|Default jerk for movement in the horizontal plane.|float|Globally
machine_max_jerk_z|Default Z Jerk|Default jerk for the motor of the Z-direction.|float|Globally
machine_max_jerk_e|Default Filament Jerk|Default jerk for the motor of the filament.|float|Globally
machine_minimum_feedrate|Minimum Feedrate|The minimal movement speed of the print head.|float|Globally
layer_height|Layer Height|The height of each layer in mm. Higher values produce faster prints in lower resolution, lower values produce slower prints in higher resolution.|float|Globally
layer_height_0|Initial Layer Height|The height of the initial layer in mm. A thicker initial layer makes adhesion to the build plate easier.|float|Globally
line_width|Line Width|Width of a single line. Generally, the width of each line should correspond to the width of the nozzle. However, slightly reducing this value could produce better prints.|float|Globally<br/>Mesh<br/>Extruder
wall_line_width|Wall Line Width|Width of a single wall line.|float|Globally<br/>Mesh<br/>Extruder
wall_line_width_0|Outer Wall Line Width|Width of the outermost wall line. By lowering this value, higher levels of detail can be printed.|float|Globally<br/>Mesh<br/>Extruder
wall_line_width_x|Inner Wall(s) Line Width|Width of a single wall line for all wall lines except the outermost one.|float|Globally<br/>Mesh<br/>Extruder
skin_line_width|Top/Bottom Line Width|Width of a single top/bottom line.|float|Globally<br/>Mesh<br/>Extruder
infill_line_width|Infill Line Width|Width of a single infill line.|float|Globally<br/>Mesh<br/>Extruder
skirt_brim_line_width|Skirt/Brim Line Width|Width of a single skirt or brim line.|float|Globally<br/>Extruder
support_line_width|Support Line Width|Width of a single support structure line.|float|Globally<br/>Extruder
support_interface_line_width|Support Interface Line Width|Width of a single support interface line.|float|Globally<br/>Extruder
prime_tower_line_width|Prime Tower Line Width|Width of a single prime tower line.|float|Globally<br/>Extruder
wall_thickness|Wall Thickness|The thickness of the outside walls in the horizontal direction. This value divided by the wall line width defines the number of walls.|float|Globally<br/>Mesh<br/>Extruder
wall_line_count|Wall Line Count|The number of walls. When calculated by the wall thickness, this value is rounded to a whole number.|int|Globally<br/>Mesh<br/>Extruder
top_bottom_thickness|Top/Bottom Thickness|The thickness of the top/bottom layers in the print. This value divided by the layer height defines the number of top/bottom layers.|float|Globally<br/>Mesh<br/>Extruder
top_thickness|Top Thickness|The thickness of the top layers in the print. This value divided by the layer height defines the number of top layers.|float|Globally<br/>Mesh<br/>Extruder
top_layers|Top Layers|The number of top layers. When calculated by the top thickness, this value is rounded to a whole number.|int|Globally<br/>Mesh<br/>Extruder
bottom_thickness|Bottom Thickness|The thickness of the bottom layers in the print. This value divided by the layer height defines the number of bottom layers.|float|Globally<br/>Mesh<br/>Extruder
bottom_layers|Bottom Layers|The number of bottom layers. When calculated by the bottom thickness, this value is rounded to a whole number.|int|Globally<br/>Mesh<br/>Extruder
top_bottom_pattern|Top/Bottom Pattern|The pattern of the top/bottom layers.|enum|Globally<br/>Mesh<br/>Extruder
wall_0_inset|Outer Wall Inset|Inset applied to the path of the outer wall. If the outer wall is smaller than the nozzle, and printed after the inner walls, use this offset to get the hole in the nozzle to overlap with the inner walls instead of the outside of the model.|float|Globally<br/>Mesh<br/>Extruder
outer_inset_first|Outer Before Inner Walls|Prints walls in order of outside to inside when enabled. This can help improve dimensional accuracy in X and Y when using a high viscosity plastic like ABS; however it can decrease outer surface print quality, especially on overhangs.|bool|Globally<br/>Mesh<br/>Extruder
alternate_extra_perimeter|Alternate Extra Wall|Prints an extra wall at every other layer. This way infill gets caught between these extra walls, resulting in stronger prints.|bool|Globally<br/>Mesh<br/>Extruder
travel_compensate_overlapping<br/>_walls_enabled|Compensate Wall Overlaps|Compensate the flow for parts of a wall being printed where there is already a wall in place.|bool|Globally<br/>Mesh<br/>Extruder
travel_compensate_overlapping<br/>_walls_0_enabled|Compensate Outer Wall Overlaps|Compensate the flow for parts of an outer wall being printed where there is already a wall in place.|bool|Globally<br/>Mesh<br/>Extruder
travel_compensate_overlapping<br/>_walls_x_enabled|Compensate Inner Wall Overlaps|Compensate the flow for parts of an inner wall being printed where there is already a wall in place.|bool|Globally<br/>Mesh<br/>Extruder
xy_offset|Horizontal Expansion|Amount of offset applied to all polygons in each layer. Positive values can compensate for too big holes; negative values can compensate for too small holes.|float|Globally<br/>Mesh<br/>Extruder
z_seam_type|Z Seam Alignment|Starting point of each path in a layer. When paths in consecutive layers start at the same point a vertical seam may show on the print. When aligning these at the back, the seam is easiest to remove. When placed randomly the inaccuracies at the paths' start will be less noticeable. When taking the shortest path the print will be quicker.|enum|Globally<br/>Mesh<br/>Extruder
skin_no_small_gaps_heuristic|Ignore Small Z Gaps|When the model has small vertical gaps, about 5% extra computation time can be spent on generating top and bottom skin in these narrow spaces. In such case, disable the setting.|bool|Globally<br/>Mesh<br/>Extruder
infill_sparse_density|Infill Density|Adjusts the density of infill of the print.|float|Globally<br/>Mesh<br/>Extruder
infill_line_distance|Infill Line Distance|Distance between the printed infill lines. This setting is calculated by the infill density and the infill line width.|float|Globally<br/>Mesh<br/>Extruder
infill_pattern|Infill Pattern|The pattern of the infill material of the print. The line and zig zag infill swap direction on alternate layers, reducing material cost. The grid, triangle, cubic, tetrahedral and concentric patterns are fully printed every layer. Cubic and tetrahedral infill change with every layer to provide a more equal distribution of strength over each direction.|enum|Globally<br/>Mesh<br/>Extruder
infill_overlap|Infill Overlap Percentage|The amount of overlap between the infill and the walls. A slight overlap allows the walls to connect firmly to the infill.|float|Globally<br/>Mesh<br/>Extruder
infill_overlap_mm|Infill Overlap|The amount of overlap between the infill and the walls. A slight overlap allows the walls to connect firmly to the infill.|float|Globally<br/>Mesh<br/>Extruder
skin_overlap|Skin Overlap Percentage|The amount of overlap between the skin and the walls. A slight overlap allows the walls to connect firmly to the skin.|float|Globally<br/>Mesh<br/>Extruder
skin_overlap_mm|Skin Overlap|The amount of overlap between the skin and the walls. A slight overlap allows the walls to connect firmly to the skin.|float|Globally<br/>Mesh<br/>Extruder
infill_wipe_dist|Infill Wipe Distance|Distance of a travel move inserted after every infill line, to make the infill stick to the walls better. This option is similar to infill overlap, but without extrusion and only on one end of the infill line.|float|Globally<br/>Mesh<br/>Extruder
infill_sparse_thickness|Infill Layer Thickness|The thickness per layer of infill material. This value should always be a multiple of the layer height and is otherwise rounded.|float|Globally<br/>Mesh<br/>Extruder
gradual_infill_steps|Gradual Infill Steps|Number of times to reduce the infill density by half when getting further below top surfaces. Areas which are closer to top surfaces get a higher density, up to the Infill Density.|int|Globally<br/>Mesh<br/>Extruder
gradual_infill_step_height|Gradual Infill Step Height|The height of infill of a given density before switching to half the density.|float|Globally<br/>Mesh<br/>Extruder
infill_before_walls|Infill Before Walls|Print the infill before printing the walls. Printing the walls first may lead to more accurate walls, but overhangs print worse. Printing the infill first leads to sturdier walls, but the infill pattern might sometimes show through the surface.|bool|Globally<br/>Mesh<br/>Extruder
material_flow_dependent<br/>_temperature|Auto Temperature|Change the temperature for each layer automatically with the average flow speed of that layer.|bool|Globally<br/>Extruder
material_print_temperature|Printing Temperature|The temperature used for printing. Set at 0 to pre-heat the printer manually.|float|Globally<br/>Extruder
material_flow_temp_graph|Flow Temperature Graph|Data linking material flow (in mm3 per second) to temperature (degrees Celsius).|str|Globally<br/>Extruder
material_extrusion_cool_down<br/>_speed|Extrusion Cool Down Speed Modifier|The extra speed by which the nozzle cools while extruding. The same value is used to signify the heat up speed lost when heating up while extruding.|float|Globally<br/>Extruder
material_bed_temperature|Build Plate Temperature|The temperature used for the heated build plate. Set at 0 to pre-heat the printer manually.|float|Globally
material_diameter|Diameter|Adjusts the diameter of the filament used. Match this value with the diameter of the used filament.|float|Globally<br/>Extruder
material_flow|Flow|Flow compensation: the amount of material extruded is multiplied by this value.|float|Globally<br/>Mesh<br/>Extruder
retraction_enable|Enable Retraction|Retract the filament when the nozzle is moving over a non-printed area. |bool|Globally<br/>Extruder
retraction_amount|Retraction Distance|The length of material retracted during a retraction move.|float|Globally<br/>Extruder
retraction_speed|Retraction Speed|The speed at which the filament is retracted and primed during a retraction move.|float|Globally<br/>Extruder
retraction_retract_speed|Retraction Retract Speed|The speed at which the filament is retracted during a retraction move.|float|Globally<br/>Extruder
retraction_prime_speed|Retraction Prime Speed|The speed at which the filament is primed during a retraction move.|float|Globally<br/>Extruder
retraction_extra_prime_amount|Retraction Extra Prime Amount|Some material can ooze away during a travel move, which can be compensated for here.|float|Globally<br/>Extruder
retraction_min_travel|Retraction Minimum Travel|The minimum distance of travel needed for a retraction to happen at all. This helps to get fewer retractions in a small area.|float|Globally<br/>Extruder
retraction_count_max|Maximum Retraction Count|This setting limits the number of retractions occurring within the minimum extrusion distance window. Further retractions within this window will be ignored. This avoids retracting repeatedly on the same piece of filament, as that can flatten the filament and cause grinding issues.|int|Globally<br/>Extruder
retraction_extrusion_window|Minimum Extrusion Distance Window|The window in which the maximum retraction count is enforced. This value should be approximately the same as the retraction distance, so that effectively the number of times a retraction passes the same patch of material is limited.|float|Globally<br/>Extruder
retraction_hop_enabled|Z Hop when Retracted|Whenever a retraction is done, the build plate is lowered to create clearance between the nozzle and the print. It prevents the nozzle from hitting the print during travel moves, reducing the chance to knock the print from the build plate.|bool|Globally<br/>Extruder
retraction_hop_only_when_collides|Z Hop Only Over Printed Parts|Only perform a Z Hop when moving over printed parts which cannot be avoided by horizontal motion by Avoid Printed Parts when Traveling.|bool|Globally<br/>Extruder
retraction_hop|Z Hop Height|The height difference when performing a Z Hop.|float|Globally<br/>Extruder
material_standby_temperature|Standby Temperature|The temperature of the nozzle when another nozzle is currently used for printing.|float|Globally<br/>Extruder
switch_extruder_retraction_amount|Nozzle Switch Retraction Distance|The amount of retraction: Set at 0 for no retraction at all. This should generally be the same as the length of the heat zone.|float|Globally<br/>Extruder
switch_extruder_retraction_speeds|Nozzle Switch Retraction Speed|The speed at which the filament is retracted. A higher retraction speed works better, but a very high retraction speed can lead to filament grinding.|float|Globally<br/>Extruder
switch_extruder_retraction_speed|Nozzle Switch Retract Speed|The speed at which the filament is retracted during a nozzle switch retract.|float|Globally<br/>Extruder
switch_extruder_prime_speed|Nozzle Switch Prime Speed|The speed at which the filament is pushed back after a nozzle switch retraction.|float|Globally<br/>Extruder
retraction_hop_after_extruder_switch|Z Hop After Extruder Switch|After the machine switched from one extruder to the other, the build plate is lowered to create clearance between the nozzle and the print. This prevents the nozzle from leaving oozed material on the outside of a print.|bool|Globally<br/>Extruder
speed_print|Print Speed|The speed at which printing happens.|float|Globally<br/>Mesh<br/>Extruder
speed_infill|Infill Speed|The speed at which infill is printed.|float|Globally<br/>Mesh<br/>Extruder
speed_wall|Wall Speed|The speed at which the walls are printed.|float|Globally<br/>Mesh<br/>Extruder
speed_wall_0|Outer Wall Speed|The speed at which the outermost walls are printed. Printing the outer wall at a lower speed improves the final skin quality. However, having a large difference between the inner wall speed and the outer wall speed will affect quality in a negative way.|float|Globally<br/>Mesh<br/>Extruder
speed_wall_x|Inner Wall Speed|The speed at which all inner walls are printed. Printing the inner wall faster than the outer wall will reduce printing time. It works well to set this in between the outer wall speed and the infill speed.|float|Globally<br/>Mesh<br/>Extruder
speed_topbottom|Top/Bottom Speed|The speed at which top/bottom layers are printed.|float|Globally<br/>Mesh<br/>Extruder
speed_support|Support Speed|The speed at which the support structure is printed. Printing support at higher speeds can greatly reduce printing time. The surface quality of the support structure is not important since it is removed after printing.|float|Globally<br/>Extruder
speed_support_infill|Support Infill Speed|The speed at which the infill of support is printed. Printing the infill at lower speeds improves stability.|float|Globally<br/>Extruder
speed_support_interface|Support Interface Speed|The speed at which the roofs and bottoms of support are printed. Printing the them at lower speeds can improve overhang quality.|float|Globally<br/>Extruder
speed_prime_tower|Prime Tower Speed|The speed at which the prime tower is printed. Printing the prime tower slower can make it more stable when the adhesion between the different filaments is suboptimal.|float|Globally<br/>Extruder
speed_travel|Travel Speed|The speed at which travel moves are made.|float|Globally<br/>Extruder
speed_layer_0|Initial Layer Speed|The speed for the initial layer. A lower value is advised to improve adhesion to the build plate.|float|Globally<br/>Mesh<br/>Extruder
speed_print_layer_0|Initial Layer Print Speed|The speed of printing for the initial layer. A lower value is advised to improve adhesion to the build plate.|float|Globally<br/>Mesh<br/>Extruder
speed_travel_layer_0|Initial Layer Travel Speed|The speed of travel moves in the initial layer. A lower value is advised to prevent pulling previously printed parts away from the build plate.|float|Globally<br/>Extruder
skirt_brim_speed|Skirt/Brim Speed|The speed at which the skirt and brim are printed. Normally this is done at the initial layer speed, but sometimes you might want to print the skirt or brim at a different speed.|float|Globally<br/>Extruder
max_feedrate_z_override|Maximum Z Speed|The maximum speed with which the build plate is moved. Setting this to zero causes the print to use the firmware defaults for the maximum z speed.|float|Globally<br/>Extruder
speed_slowdown_layers|Number of Slower Layers|The first few layers are printed slower than the rest of the model, to get better adhesion to the build plate and improve the overall success rate of prints. The speed is gradually increased over these layers.|int|Globally
speed_equalize_flow_enabled|Equalize Filament Flow|Print thinner than normal lines faster so that the amount of material extruded per second remains the same. Thin pieces in your model might require lines printed with smaller line width than provided in the settings. This setting controls the speed changes for such lines.|bool|Globally<br/>Extruder
speed_equalize_flow_max|Maximum Speed for Flow Equalization|Maximum print speed when adjusting the print speed in order to equalize flow.|float|Globally<br/>Extruder
acceleration_enabled|Enable Acceleration Control|Enables adjusting the print head acceleration. Increasing the accelerations can reduce printing time at the cost of print quality.|bool|Globally
acceleration_print|Print Acceleration|The acceleration with which printing happens.|float|Globally<br/>Mesh<br/>Extruder
acceleration_infill|Infill Acceleration|The acceleration with which infill is printed.|float|Globally<br/>Mesh<br/>Extruder
acceleration_wall|Wall Acceleration|The acceleration with which the walls are printed.|float|Globally<br/>Mesh<br/>Extruder
acceleration_wall_0|Outer Wall Acceleration|The acceleration with which the outermost walls are printed.|float|Globally<br/>Mesh<br/>Extruder
acceleration_wall_x|Inner Wall Acceleration|The acceleration with which all inner walls are printed.|float|Globally<br/>Mesh<br/>Extruder
acceleration_topbottom|Top/Bottom Acceleration|The acceleration with which top/bottom layers are printed.|float|Globally<br/>Mesh<br/>Extruder
acceleration_support|Support Acceleration|The acceleration with which the support structure is printed.|float|Globally<br/>Extruder
acceleration_support_infill|Support Infill Acceleration|The acceleration with which the infill of support is printed.|float|Globally<br/>Extruder
acceleration_support_interface|Support Interface Acceleration|The acceleration with which the roofs and bottoms of support are printed. Printing them at lower accelerations can improve overhang quality.|float|Globally<br/>Extruder
acceleration_prime_tower|Prime Tower Acceleration|The acceleration with which the prime tower is printed.|float|Globally<br/>Extruder
acceleration_travel|Travel Acceleration|The acceleration with which travel moves are made.|float|Globally<br/>Extruder
acceleration_layer_0|Initial Layer Acceleration|The acceleration for the initial layer.|float|Globally<br/>Mesh<br/>Extruder
acceleration_print_layer_0|Initial Layer Print Acceleration|The acceleration during the printing of the initial layer.|float|Globally<br/>Mesh<br/>Extruder
acceleration_travel_layer_0|Initial Layer Travel Acceleration|The acceleration for travel moves in the initial layer.|float|Globally<br/>Extruder
acceleration_skirt_brim|Skirt/Brim Acceleration|The acceleration with which the skirt and brim are printed. Normally this is done with the initial layer acceleration, but sometimes you might want to print the skirt or brim at a different acceleration.|float|Globally<br/>Extruder
jerk_enabled|Enable Jerk Control|Enables adjusting the jerk of print head when the velocity in the X or Y axis changes. Increasing the jerk can reduce printing time at the cost of print quality.|bool|Globally
jerk_print|Print Jerk|The maximum instantaneous velocity change of the print head.|float|Globally<br/>Mesh<br/>Extruder
jerk_infill|Infill Jerk|The maximum instantaneous velocity change with which infill is printed.|float|Globally<br/>Mesh<br/>Extruder
jerk_wall|Wall Jerk|The maximum instantaneous velocity change with which the walls are printed.|float|Globally<br/>Mesh<br/>Extruder
jerk_wall_0|Outer Wall Jerk|The maximum instantaneous velocity change with which the outermost walls are printed.|float|Globally<br/>Mesh<br/>Extruder
jerk_wall_x|Inner Wall Jerk|The maximum instantaneous velocity change with which all inner walls are printed.|float|Globally<br/>Mesh<br/>Extruder
jerk_topbottom|Top/Bottom Jerk|The maximum instantaneous velocity change with which top/bottom layers are printed.|float|Globally<br/>Mesh<br/>Extruder
jerk_support|Support Jerk|The maximum instantaneous velocity change with which the support structure is printed.|float|Globally<br/>Extruder
jerk_support_infill|Support Infill Jerk|The maximum instantaneous velocity change with which the infill of support is printed.|float|Globally<br/>Extruder
jerk_support_interface|Support Interface Jerk|The maximum instantaneous velocity change with which the roofs and bottoms of support are printed.|float|Globally<br/>Extruder
jerk_prime_tower|Prime Tower Jerk|The maximum instantaneous velocity change with which the prime tower is printed.|float|Globally<br/>Extruder
jerk_travel|Travel Jerk|The maximum instantaneous velocity change with which travel moves are made.|float|Globally<br/>Extruder
jerk_layer_0|Initial Layer Jerk|The print maximum instantaneous velocity change for the initial layer.|float|Globally<br/>Mesh<br/>Extruder
jerk_print_layer_0|Initial Layer Print Jerk|The maximum instantaneous velocity change during the printing of the initial layer.|float|Globally<br/>Mesh<br/>Extruder
jerk_travel_layer_0|Initial Layer Travel Jerk|The acceleration for travel moves in the initial layer.|float|Globally<br/>Extruder
jerk_skirt_brim|Skirt/Brim Jerk|The maximum instantaneous velocity change with which the skirt and brim are printed.|float|Globally<br/>Extruder
retraction_combing|Combing Mode|Combing keeps the nozzle within already printed areas when traveling. This results in slightly longer travel moves but reduces the need for retractions. If combing is off, the material will retract and the nozzle moves in a straight line to the next point. It is also possible to avoid combing over top/bottom skin areas by combing within the infill only. It is also possible to avoid combing over top/bottom skin areas by combing within the infill only.|enum|Globally<br/>Mesh
travel_avoid_other_parts|Avoid Printed Parts when Traveling|The nozzle avoids already printed parts when traveling. This option is only available when combing is enabled.|bool|Globally<br/>Extruder
travel_avoid_distance|Travel Avoid Distance|The distance between the nozzle and already printed parts when avoiding during travel moves.|float|Globally<br/>Extruder
cool_fan_enabled|Enable Print Cooling|Enables the print cooling fans while printing. The fans improve print quality on layers with short layer times and bridging / overhangs.|bool|Globally<br/>Extruder
cool_fan_speed|Fan Speed|The speed at which the print cooling fans spin.|float|Globally<br/>Extruder
cool_fan_speed_min|Regular Fan Speed|The speed at which the fans spin before hitting the threshold. When a layer prints faster than the threshold, the fan speed gradually inclines towards the maximum fan speed.|float|Globally<br/>Extruder
cool_fan_speed_max|Maximum Fan Speed|The speed at which the fans spin on the minimum layer time. The fan speed gradually increases between the regular fan speed and maximum fan speed when the threshold is hit.|float|Globally<br/>Extruder
cool_min_layer_time_fan_speed_max|Regular/Maximum Fan Speed Threshold|The layer time which sets the threshold between regular fan speed and maximum fan speed. Layers that print slower than this time use regular fan speed. For faster layers the fan speed gradually increases towards the maximum fan speed.|float|Globally<br/>Extruder
cool_fan_full_at_height|Regular Fan Speed at Height|The height at which the fans spin on regular fan speed. At the layers below the fan speed gradually increases from zero to regular fan speed.|float|Globally<br/>Extruder
cool_fan_full_layer|Regular Fan Speed at Layer|The layer at which the fans spin on regular fan speed. If regular fan speed at height is set, this value is calculated and rounded to a whole number.|int|Globally<br/>Extruder
cool_min_layer_time|Minimum Layer Time|The minimum time spent in a layer. This forces the printer to slow down, to at least spend the time set here in one layer. This allows the printed material to cool down properly before printing the next layer.|float|Globally<br/>Extruder
cool_min_speed|Minimum Speed|The minimum print speed, despite slowing down due to the minimum layer time. When the printer would slow down too much, the pressure in the nozzle would be too low and result in bad print quality.|float|Globally<br/>Extruder
cool_lift_head|Lift Head|When the minimum speed is hit because of minimum layer time, lift the head away from the print and wait the extra time until the minimum layer time is reached.|bool|Globally<br/>Extruder
support_enable|Enable Support|Enable support structures. These structures support parts of the model with severe overhangs.|bool|Globally<br/>Mesh
support_type|Support Placement|Adjusts the placement of the support structures. The placement can be set to touching build plate or everywhere. When set to everywhere the support structures will also be printed on the model.|enum|Globally
support_angle|Support Overhang Angle|The minimum angle of overhangs for which support is added. At a value of 0° all overhangs are supported, 90° will not provide any support.|float|Globally<br/>Mesh<br/>Extruder
support_pattern|Support Pattern|The pattern of the support structures of the print. The different options available result in sturdy or easy to remove support.|enum|Globally<br/>Extruder
support_connect_zigzags|Connect Support ZigZags|Connect the ZigZags. This will increase the strength of the zig zag support structure.|bool|Globally<br/>Extruder
support_infill_rate|Support Density|Adjusts the density of the support structure. A higher value results in better overhangs, but the supports are harder to remove.|float|Globally<br/>Extruder
support_line_distance|Support Line Distance|Distance between the printed support structure lines. This setting is calculated by the support density.|float|Globally<br/>Extruder
support_z_distance|Support Z Distance|Distance from the top/bottom of the support structure to the print. This gap provides clearance to remove the supports after the model is printed. This value is rounded down to a multiple of the layer height.|float|Globally<br/>Mesh<br/>Extruder
support_top_distance|Support Top Distance|Distance from the top of the support to the print.|float|Globally<br/>Mesh<br/>Extruder
support_bottom_distance|Support Bottom Distance|Distance from the print to the bottom of the support.|float|Globally<br/>Mesh<br/>Extruder
support_xy_distance|Support X/Y Distance|Distance of the support structure from the print in the X/Y directions.|float|Globally<br/>Mesh<br/>Extruder
support_xy_overrides_z|Support Distance Priority|Whether the Support X/Y Distance overrides the Support Z Distance or vice versa. When X/Y overrides Z the X/Y distance can push away the support from the model, influencing the actual Z distance to the overhang. We can disable this by not applying the X/Y distance around overhangs.|enum|Globally<br/>Mesh<br/>Extruder
support_xy_distance_overhang|Minimum Support X/Y Distance|Distance of the support structure from the overhang in the X/Y directions. |float|Globally<br/>Mesh<br/>Extruder
support_bottom_stair_step_height|Support Stair Step Height|The height of the steps of the stair-like bottom of support resting on the model. A low value makes the support harder to remove, but too high values can lead to unstable support structures.|float|Globally<br/>Mesh<br/>Extruder
support_join_distance|Support Join Distance|The maximum distance between support structures in the X/Y directions. When seperate structures are closer together than this value, the structures merge into one.|float|Globally<br/>Mesh<br/>Extruder
support_offset|Support Horizontal Expansion|Amount of offset applied to all support polygons in each layer. Positive values can smooth out the support areas and result in more sturdy support.|float|Globally<br/>Mesh<br/>Extruder
support_interface_enable|Enable Support Interface|Generate a dense interface between the model and the support. This will create a skin at the top of the support on which the model is printed and at the bottom of the support, where it rests on the model.|bool|Globally<br/>Mesh<br/>Extruder
support_interface_height|Support Interface Thickness|The thickness of the interface of the support where it touches with the model on the bottom or the top.|float|Globally<br/>Mesh<br/>Extruder
support_roof_height|Support Roof Thickness|The thickness of the support roofs. This controls the amount of dense layers at the top of the support on which the model rests.|float|Globally<br/>Mesh<br/>Extruder
support_bottom_height|Support Bottom Thickness|The thickness of the support bottoms. This controls the number of dense layers are printed on top of places of a model on which support rests.|float|Globally<br/>Mesh<br/>Extruder
support_interface_skip_height|Support Interface Resolution|When checking where there's model above the support, take steps of the given height. Lower values will slice slower, while higher values may cause normal support to be printed in some places where there should have been support interface.|float|Globally<br/>Mesh<br/>Extruder
support_interface_density|Support Interface Density|Adjusts the density of the roofs and bottoms of the support structure. A higher value results in better overhangs, but the supports are harder to remove.|float|Globally<br/>Extruder
support_interface_line_distance|Support Interface Line Distance|Distance between the printed support interface lines. This setting is calculated by the Support Interface Density, but can be adjusted separately.|float|Globally<br/>Extruder
support_interface_pattern|Support Interface Pattern|The pattern with which the interface of the support with the model is printed.|enum|Globally<br/>Extruder
support_use_towers|Use Towers|Use specialized towers to support tiny overhang areas. These towers have a larger diameter than the region they support. Near the overhang the towers' diameter decreases, forming a roof.|bool|Globally<br/>Mesh<br/>Extruder
support_tower_diameter|Tower Diameter|The diameter of a special tower.|float|Globally<br/>Mesh<br/>Extruder
support_minimal_diameter|Minimum Diameter|Minimum diameter in the X/Y directions of a small area which is to be supported by a specialized support tower.|float|Globally<br/>Mesh<br/>Extruder
support_tower_roof_angle|Tower Roof Angle|The angle of a rooftop of a tower. A higher value results in pointed tower roofs, a lower value results in flattened tower roofs.|int|Globally<br/>Mesh<br/>Extruder
extruder_prime_pos_x|Extruder Prime X Position|The X coordinate of the position where the nozzle primes at the start of printing.|float|Globally<br/>Extruder
extruder_prime_pos_y|Extruder Prime Y Position|The Y coordinate of the position where the nozzle primes at the start of printing.|float|Globally<br/>Extruder
adhesion_type|Build Plate Adhesion Type|Different options that help to improve both priming your extrusion and adhesion to the build plate. Brim adds a single layer flat area around the base of your model to prevent warping. Raft adds a thick grid with a roof below the model. Skirt is a line printed around the model, but not connected to the model.|enum|Globally
skirt_line_count|Skirt Line Count|Multiple skirt lines help to prime your extrusion better for small models. Setting this to 0 will disable the skirt.|int|Globally<br/>Extruder
skirt_gap|Skirt Distance|The horizontal distance between the skirt and the first layer of the print. This is the minimum distance, multiple skirt lines will extend outwards from this distance.|float|Globally<br/>Extruder
skirt_brim_minimal_length|Skirt/Brim Minimum Length|The minimum length of the skirt or brim. If this length is not reached by all skirt or brim lines together, more skirt or brim lines will be added until the minimum length is reached. Note: If the line count is set to 0 this is ignored.|float|Globally<br/>Extruder
brim_width|Brim Width|The distance from the model to the outermost brim line. A larger brim enhances adhesion to the build plate, but also reduces the effective print area.|float|Globally<br/>Extruder
brim_line_count|Brim Line Count|The number of lines used for a brim. More brim lines enhance adhesion to the build plate, but also reduces the effective print area.|int|Globally<br/>Extruder
brim_outside_only|Brim Only on Outside|Only print the brim on the outside of the model. This reduces the amount of brim you need to remove afterwards, while it doesn't reduce the bed adhesion that much.|bool|Globally<br/>Extruder
raft_margin|Raft Extra Margin|If the raft is enabled, this is the extra raft area around the model which is also given a raft. Increasing this margin will create a stronger raft while using more material and leaving less area for your print.|float|Globally<br/>Extruder
raft_airgap|Raft Air Gap|The gap between the final raft layer and the first layer of the model. Only the first layer is raised by this amount to lower the bonding between the raft layer and the model. Makes it easier to peel off the raft.|float|Globally<br/>Extruder
layer_0_z_overlap|Initial Layer Z Overlap|Make the first and second layer of the model overlap in the Z direction to compensate for the filament lost in the airgap. All models above the first model layer will be shifted down by this amount.|float|Globally<br/>Extruder
raft_surface_layers|Raft Top Layers|The number of top layers on top of the 2nd raft layer. These are fully filled layers that the model sits on. 2 layers result in a smoother top surface than 1.|int|Globally<br/>Extruder
raft_surface_thickness|Raft Top Layer Thickness|Layer thickness of the top raft layers.|float|Globally<br/>Extruder
raft_surface_line_width|Raft Top Line Width|Width of the lines in the top surface of the raft. These can be thin lines so that the top of the raft becomes smooth.|float|Globally<br/>Extruder
raft_surface_line_spacing|Raft Top Spacing|The distance between the raft lines for the top raft layers. The spacing should be equal to the line width, so that the surface is solid.|float|Globally<br/>Extruder
raft_interface_thickness|Raft Middle Thickness|Layer thickness of the middle raft layer.|float|Globally<br/>Extruder
raft_interface_line_width|Raft Middle Line Width|Width of the lines in the middle raft layer. Making the second layer extrude more causes the lines to stick to the build plate.|float|Globally<br/>Extruder
raft_interface_line_spacing|Raft Middle Spacing|The distance between the raft lines for the middle raft layer. The spacing of the middle should be quite wide, while being dense enough to support the top raft layers.|float|Globally<br/>Extruder
raft_base_thickness|Raft Base Thickness|Layer thickness of the base raft layer. This should be a thick layer which sticks firmly to the printer build plate.|float|Globally<br/>Extruder
raft_base_line_width|Raft Base Line Width|Width of the lines in the base raft layer. These should be thick lines to assist in build plate adhesion.|float|Globally<br/>Extruder
raft_base_line_spacing|Raft Line Spacing|The distance between the raft lines for the base raft layer. Wide spacing makes for easy removal of the raft from the build plate.|float|Globally<br/>Extruder
raft_speed|Raft Print Speed|The speed at which the raft is printed.|float|Globally<br/>Extruder
raft_surface_speed|Raft Top Print Speed|The speed at which the top raft layers are printed. These should be printed a bit slower, so that the nozzle can slowly smooth out adjacent surface lines.|float|Globally<br/>Extruder
raft_interface_speed|Raft Middle Print Speed|The speed at which the middle raft layer is printed. This should be printed quite slowly, as the volume of material coming out of the nozzle is quite high.|float|Globally<br/>Extruder
raft_base_speed|Raft Base Print Speed|The speed at which the base raft layer is printed. This should be printed quite slowly, as the volume of material coming out of the nozzle is quite high.|float|Globally<br/>Extruder
raft_acceleration|Raft Print Acceleration|The acceleration with which the raft is printed.|float|Globally<br/>Extruder
raft_surface_acceleration|Raft Top Print Acceleration|The acceleration with which the top raft layers are printed.|float|Globally<br/>Extruder
raft_interface_acceleration|Raft Middle Print Acceleration|The acceleration with which the middle raft layer is printed.|float|Globally<br/>Extruder
raft_base_acceleration|Raft Base Print Acceleration|The acceleration with which the base raft layer is printed.|float|Globally<br/>Extruder
raft_jerk|Raft Print Jerk|The jerk with which the raft is printed.|float|Globally<br/>Extruder
raft_surface_jerk|Raft Top Print Jerk|The jerk with which the top raft layers are printed.|float|Globally<br/>Extruder
raft_interface_jerk|Raft Middle Print Jerk|The jerk with which the middle raft layer is printed.|float|Globally<br/>Extruder
raft_base_jerk|Raft Base Print Jerk|The jerk with which the base raft layer is printed.|float|Globally<br/>Extruder
raft_fan_speed|Raft Fan Speed|The fan speed for the raft.|float|Globally<br/>Extruder
raft_surface_fan_speed|Raft Top Fan Speed|The fan speed for the top raft layers.|float|Globally<br/>Extruder
raft_interface_fan_speed|Raft Middle Fan Speed|The fan speed for the middle raft layer.|float|Globally<br/>Extruder
raft_base_fan_speed|Raft Base Fan Speed|The fan speed for the base raft layer.|float|Globally<br/>Extruder
adhesion_extruder_nr|Build Plate Adhesion Extruder|The extruder train to use for printing the skirt/brim/raft. This is used in multi-extrusion.|extruder|Globally
support_extruder_nr|Support Extruder|The extruder train to use for printing the support. This is used in multi-extrusion.|extruder|Globally
support_infill_extruder_nr|Support Infill Extruder|The extruder train to use for printing the infill of the support. This is used in multi-extrusion.|extruder|Globally
support_extruder_nr_layer_0|First Layer Support Extruder|The extruder train to use for printing the first layer of support infill. This is used in multi-extrusion.|extruder|Globally
support_interface_extruder_nr|Support Interface Extruder|The extruder train to use for printing the roofs and bottoms of the support. This is used in multi-extrusion.|extruder|Globally
prime_tower_enable|Enable Prime Tower|Print a tower next to the print which serves to prime the material after each nozzle switch.|bool|Globally
prime_tower_size|Prime Tower Size|The width of the prime tower.|float|Globally
prime_tower_position_x|Prime Tower X Position|The x coordinate of the position of the prime tower.|float|Globally
prime_tower_position_y|Prime Tower Y Position|The y coordinate of the position of the prime tower.|float|Globally
prime_tower_flow|Prime Tower Flow|Flow compensation: the amount of material extruded is multiplied by this value.|float|Globally<br/>Extruder
prime_tower_wipe_enabled|Wipe Nozzle on Prime Tower|After printing the prime tower with one nozzle, wipe the oozed material from the other nozzle off on the prime tower.|bool|Globally
multiple_mesh_overlap|Dual Extrusion Overlap|Make the models printed with different extruder trains overlap a bit. This makes the different materials bond together better.|float|Globally<br/>Mesh<br/>Extruder
ooze_shield_enabled|Enable Ooze Shield|Enable exterior ooze shield. This will create a shell around the model which is likely to wipe a second nozzle if it's at the same height as the first nozzle.|bool|Globally
ooze_shield_angle|Ooze Shield Angle|The maximum angle a part in the ooze shield will have. With 0 degrees being vertical, and 90 degrees being horizontal. A smaller angle leads to less failed ooze shields, but more material.|float|Globally
ooze_shield_dist|Ooze Shield Distance|Distance of the ooze shield from the print, in the X/Y directions.|float|Globally
meshfix_union_all|Union Overlapping Volumes|Ignore the internal geometry arising from overlapping volumes and print the volumes as one. This may cause internal cavities to disappear.|bool|Globally<br/>Mesh<br/>Extruder
meshfix_union_all_remove_holes|Remove All Holes|Remove the holes in each layer and keep only the outside shape. This will ignore any invisible internal geometry. However, it also ignores layer holes which can be viewed from above or below.|bool|Globally<br/>Mesh<br/>Extruder
meshfix_extensive_stitching|Extensive Stitching|Extensive stitching tries to stitch up open holes in the mesh by closing the hole with touching polygons. This option can introduce a lot of processing time.|bool|Globally<br/>Mesh<br/>Extruder
meshfix_keep_open_polygons|Keep Disconnected Faces|Normally Cura tries to stitch up small holes in the mesh and remove parts of a layer with big holes. Enabling this option keeps those parts which cannot be stitched. This option should be used as a last resort option when everything else fails to produce proper GCode.|bool|Globally<br/>Mesh<br/>Extruder
print_sequence|Print Sequence|Whether to print all models one layer at a time or to wait for one model to finish, before moving on to the next. One at a time mode is only possible if all models are separated in such a way that the whole print head can move in between and all models are lower than the distance between the nozzle and the X/Y axes.|enum|Globally
infill_mesh|Infill Mesh|Use this mesh to modify the infill of other meshes with which it overlaps. Replaces infill regions of other meshes with regions for this mesh. It's suggested to only print one Wall and no Top/Bottom Skin for this mesh.|bool|Mesh
infill_mesh_order|Infill Mesh Order|Determines which infill mesh is inside the infill of another infill mesh. An infill mesh with a higher order will modify the infill of infill meshes with lower order and normal meshes.|int|Mesh
magic_mesh_surface_mode|Surface Mode|Treat the model as a surface only, a volume, or volumes with loose surfaces. The normal print mode only prints enclosed volumes. "Surface" prints a single wall tracing the mesh surface with no infill and no top/bottom skin. "Both" prints enclosed volumes like normal and any remaining polygons as surfaces.|enum|Globally<br/>Mesh<br/>Extruder
magic_spiralize|Spiralize Outer Contour|Spiralize smooths out the Z move of the outer edge. This will create a steady Z increase over the whole print. This feature turns a solid model into a single walled print with a solid bottom. This feature used to be called Joris in older versions.|bool|Globally<br/>Mesh<br/>Extruder
draft_shield_enabled|Enable Draft Shield|This will create a wall around the model, which traps (hot) air and shields against exterior airflow. Especially useful for materials which warp easily.|bool|Globally
draft_shield_dist|Draft Shield X/Y Distance|Distance of the draft shield from the print, in the X/Y directions.|float|Globally
draft_shield_height_limitation|Draft Shield Limitation|Set the height of the draft shield. Choose to print the draft shield at the full height of the model or at a limited height.|enum|Globally
draft_shield_height|Draft Shield Height|Height limitation of the draft shield. Above this height no draft shield will be printed.|float|Globally
conical_overhang_enabled|Make Overhang Printable|Change the geometry of the printed model such that minimal support is required. Steep overhangs will become shallow overhangs. Overhanging areas will drop down to become more vertical.|bool|Globally<br/>Extruder
conical_overhang_angle|Maximum Model Angle|The maximum angle of overhangs after the they have been made printable. At a value of 0° all overhangs are replaced by a piece of model connected to the build plate, 90° will not change the model in any way.|float|Globally<br/>Extruder
coasting_enable|Enable Coasting|Coasting replaces the last part of an extrusion path with a travel path. The oozed material is used to print the last piece of the extrusion path in order to reduce stringing.|bool|Globally<br/>Extruder
coasting_volume|Coasting Volume|The volume otherwise oozed. This value should generally be close to the nozzle diameter cubed.|float|Globally<br/>Extruder
coasting_min_volume|Minimum Volume Before Coasting|The smallest volume an extrusion path should have before allowing coasting. For smaller extrusion paths, less pressure has been built up in the bowden tube and so the coasted volume is scaled linearly. This value should always be larger than the Coasting Volume.|float|Globally<br/>Extruder
coasting_speed|Coasting Speed|The speed by which to move during coasting, relative to the speed of the extrusion path. A value slightly under 100% is advised, since during the coasting move the pressure in the bowden tube drops.|float|Globally<br/>Extruder
skin_outline_count|Extra Skin Wall Count|Replaces the outermost part of the top/bottom pattern with a number of concentric lines. Using one or two lines improves roofs that start on infill material.|int|Globally<br/>Mesh<br/>Extruder
skin_alternate_rotation|Alternate Skin Rotation|Alternate the direction in which the top/bottom layers are printed. Normally they are printed diagonally only. This setting adds the X-only and Y-only directions.|bool|Globally<br/>Mesh<br/>Extruder
support_conical_enabled|Enable Conical Support|Experimental feature: Make support areas smaller at the bottom than at the overhang.|bool|Globally<br/>Mesh<br/>Extruder
support_conical_angle|Conical Support Angle|The angle of the tilt of conical support. With 0 degrees being vertical, and 90 degrees being horizontal. Smaller angles cause the support to be more sturdy, but consist of more material. Negative angles cause the base of the support to be wider than the top.|float|Globally<br/>Mesh<br/>Extruder
support_conical_min_width|Conical Support Minimum Width|Minimum width to which the base of the conical support area is reduced. Small widths can lead to unstable support structures.|float|Globally<br/>Mesh<br/>Extruder
magic_fuzzy_skin_enabled|Fuzzy Skin|Randomly jitter while printing the outer wall, so that the surface has a rough and fuzzy look.|bool|Globally<br/>Mesh<br/>Extruder
magic_fuzzy_skin_thickness|Fuzzy Skin Thickness|The width within which to jitter. It's advised to keep this below the outer wall width, since the inner walls are unaltered.|float|Globally<br/>Mesh<br/>Extruder
magic_fuzzy_skin_point_density|Fuzzy Skin Density|The average density of points introduced on each polygon in a layer. Note that the original points of the polygon are discarded, so a low density results in a reduction of the resolution.|float|Globally<br/>Mesh<br/>Extruder
magic_fuzzy_skin_point_dist|Fuzzy Skin Point Distance|The average distance between the random points introduced on each line segment. Note that the original points of the polygon are discarded, so a high smoothness results in a reduction of the resolution. This value must be higher than half the Fuzzy Skin Thickness.|float|Globally<br/>Mesh<br/>Extruder
wireframe_enabled|Wire Printing|Print only the outside surface with a sparse webbed structure, printing 'in thin air'. This is realized by horizontally printing the contours of the model at given Z intervals which are connected via upward and diagonally downward lines.|bool|Globally
wireframe_height|WP Connection Height|The height of the upward and diagonally downward lines between two horizontal parts. This determines the overall density of the net structure. Only applies to Wire Printing.|float|Globally
wireframe_roof_inset|WP Roof Inset Distance|The distance covered when making a connection from a roof outline inward. Only applies to Wire Printing.|float|Globally
wireframe_printspeed|WP Speed|Speed at which the nozzle moves when extruding material. Only applies to Wire Printing.|float|Globally
wireframe_printspeed_bottom|WP Bottom Printing Speed|Speed of printing the first layer, which is the only layer touching the build platform. Only applies to Wire Printing.|float|Globally
wireframe_printspeed_up|WP Upward Printing Speed|Speed of printing a line upward 'in thin air'. Only applies to Wire Printing.|float|Globally
wireframe_printspeed_down|WP Downward Printing Speed|Speed of printing a line diagonally downward. Only applies to Wire Printing.|float|Globally
wireframe_printspeed_flat|WP Horizontal Printing Speed|Speed of printing the horizontal contours of the model. Only applies to Wire Printing.|float|Globally
wireframe_flow|WP Flow|Flow compensation: the amount of material extruded is multiplied by this value. Only applies to Wire Printing.|float|Globally
wireframe_flow_connection|WP Connection Flow|Flow compensation when going up or down. Only applies to Wire Printing.|float|Globally
wireframe_flow_flat|WP Flat Flow|Flow compensation when printing flat lines. Only applies to Wire Printing.|float|Globally
wireframe_top_delay|WP Top Delay|Delay time after an upward move, so that the upward line can harden. Only applies to Wire Printing.|float|Globally
wireframe_bottom_delay|WP Bottom Delay|Delay time after a downward move. Only applies to Wire Printing.|float|Globally
wireframe_flat_delay|WP Flat Delay|Delay time between two horizontal segments. Introducing such a delay can cause better adhesion to previous layers at the connection points, while too long delays cause sagging. Only applies to Wire Printing.|float|Globally
wireframe_up_half_speed|WP Ease Upward|Distance of an upward move which is extruded with half speed. This can cause better adhesion to previous layers, while not heating the material in those layers too much. Only applies to Wire Printing.|float|Globally
wireframe_top_jump|WP Knot Size|Creates a small knot at the top of an upward line, so that the consecutive horizontal layer has a better chance to connect to it. Only applies to Wire Printing.|float|Globally
wireframe_fall_down|WP Fall Down|Distance with which the material falls down after an upward extrusion. This distance is compensated for. Only applies to Wire Printing.|float|Globally
wireframe_drag_along|WP Drag Along|Distance with which the material of an upward extrusion is dragged along with the diagonally downward extrusion. This distance is compensated for. Only applies to Wire Printing.|float|Globally
wireframe_strategy|WP Strategy|Strategy for making sure two consecutive layers connect at each connection point. Retraction lets the upward lines harden in the right position, but may cause filament grinding. A knot can be made at the end of an upward line to heighten the chance of connecting to it and to let the line cool; however, it may require slow printing speeds. Another strategy is to compensate for the sagging of the top of an upward line; however, the lines won't always fall down as predicted.|enum|Globally
wireframe_straight_before_down|WP Straighten Downward Lines|Percentage of a diagonally downward line which is covered by a horizontal line piece. This can prevent sagging of the top most point of upward lines. Only applies to Wire Printing.|float|Globally
wireframe_roof_fall_down|WP Roof Fall Down|The distance which horizontal roof lines printed 'in thin air' fall down when being printed. This distance is compensated for. Only applies to Wire Printing.|float|Globally
wireframe_roof_drag_along|WP Roof Drag Along|The distance of the end piece of an inward line which gets dragged along when going back to the outer outline of the roof. This distance is compensated for. Only applies to Wire Printing.|float|Globally
wireframe_roof_outer_delay|WP Roof Outer Delay|Time spent at the outer perimeters of hole which is to become a roof. Longer times can ensure a better connection. Only applies to Wire Printing.|float|Globally
wireframe_nozzle_clearance|WP Nozzle Clearance|Distance between the nozzle and horizontally downward lines. Larger clearance results in diagonally downward lines with a less steep angle, which in turn results in less upward connections with the next layer. Only applies to Wire Printing.|float|Globally

## Extruder Settings
Key|Label|Description|Type|Settable
---|-----|-----------|----|--------
extruder_nr|Extruder|The extruder train used for printing. This is used in multi-extrusion.|extruder|Mesh
machine_nozzle_offset_x|Nozzle X Offset|The x-coordinate of the offset of the nozzle.|float|Extruder
machine_nozzle_offset_y|Nozzle Y Offset|The y-coordinate of the offset of the nozzle.|float|Extruder
machine_extruder_start_code|Extruder Start G-Code|Start g-code to execute whenever turning the extruder on.|str|Extruder
machine_extruder_start_pos_abs|Extruder Start Position Absolute|Make the extruder starting position absolute rather than relative to the last-known location of the head.|bool|Extruder
machine_extruder_start_pos_x|Extruder Start Position X|The x-coordinate of the starting position when turning the extruder on.|float|Extruder
machine_extruder_start_pos_y|Extruder Start Position Y|The y-coordinate of the starting position when turning the extruder on.|float|Extruder
machine_extruder_end_code|Extruder End G-Code|End g-code to execute whenever turning the extruder off.|str|Extruder
machine_extruder_end_pos_abs|Extruder End Position Absolute|Make the extruder ending position absolute rather than relative to the last-known location of the head.|bool|Extruder
machine_extruder_end_pos_x|Extruder End Position X|The x-coordinate of the ending position when turning the extruder off.|float|Extruder
machine_extruder_end_pos_y|Extruder End Position Y|The y-coordinate of the ending position when turning the extruder off.|float|Extruder
extruder_prime_pos_z|Extruder Prime Z Position|The Z coordinate of the position where the nozzle primes at the start of printing.|float|Globally<br/>Extruder
extruder_prime_pos_x|Extruder Prime X Position|The X coordinate of the position where the nozzle primes at the start of printing.|float|Globally<br/>Extruder
extruder_prime_pos_y|Extruder Prime Y Position|The Y coordinate of the position where the nozzle primes at the start of printing.|float|Globally<br/>Extruder

## Enums
### Gcode flavour
Key|Description
---|-----------
RepRap (Marlin/Sprinter)|RepRap (Marlin/Sprinter)
RepRap (Volumatric)|RepRap (Volumetric)
UltiGCode|Ultimaker 2
Griffin|Griffin
Makerbot|Makerbot
BFB|Bits from Bytes
MACH3|Mach3
Repetier|Repetier

### Top/Bottom Pattern
Key|Description
---|-----------
lines|Lines
concentric|Concentric
zigzag|Zig Zag

### Z Seam Alignment
Key|Description
---|-----------
back|Back
shortest|Shortest
random|Random

### Infill Pattern
Key|Description
---|-----------
grid|Grid
lines|Lines
triangles|Triangles
cubic|Cubic
tetrahedral|Tetrahedral
concentric|Concentric
zigzag|Zig Zag

### Combing Mode
Key|Description
---|-----------
off|Off
all|All
noskin|No Skin

### Support Placement
Key|Description
---|-----------
buildplate|Touching Buildplate
everywhere|Everywhere

### Support Pattern
Key|Description
---|-----------
lines|Lines
grid|Grid
triangles|Triangles
concentric|Concentric
zigzag|Zig Zag

### Support Distance Priority
Key|Description
---|-----------
xy_overrides_z|X/Y overrides Z
z_overrides_xy|Z overrides X/Y

### Support Interface Pattern
Key|Description
---|-----------
lines|Lines
grid|Grid
triangles|Triangles
concentric|Concentric
zigzag|Zig Zag

### Build Plate Adhesion Type
Key|Description
---|-----------
skirt|Skirt
brim|Brim
raft|Raft

### Print Sequence
Key|Description
---|-----------
all_at_once|All at Once
one_at_a_time|One at a Time

### Surface Mode
Key|Description
---|-----------
normal|Normal
surface|Surface
both|Both

### Draft Shield Limitation
Key|Description
---|-----------
full|Full
limited|Limited

### WP Strategy
Key|Description
---|-----------
compensate|Compensate
knot|Knot
retract|Retract
