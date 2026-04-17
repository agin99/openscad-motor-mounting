include <BOSL2/std.scad>
include <BOSL2/nema_steppers.scad>

$fn = 100;

// ========== CONSTANTS ========== //

gusset_width = 5;
gusset_side = 15;
mount_screw_d = 4;
ball_bearing_od = 14;

// ========== LOGIC ========== //

function belt_l_to_z(mod_val, l, adj_gear_z) = 2 * l / mod_val - adj_gear_z;

// ========== STRUCTURES ========== //

module tolerant_screw_hole(separation, d, l) {
    hull() {
        translate([0, -separation / 2, 0])
            cylinder(d = d, h = l, center = true);
        translate([0, separation / 2, 0])
            cylinder(d = d, h = l, center = true);
    }
}

module gusset(side_length, width) {
    linear_extrude(width)
        polygon([
            [0, 0],
            [side_length, 0],
            [0, side_length]
        ]);
}

/* From BOSL2: 
    MOTOR_WIDTH:    The full width and length of the motor.
    PLINTH_HEIGHT:  The height of the circular plinth on the face of the motor.
    PLINTH_DIAM:    The diameter of the circular plinth on the face of the motor.
    SCREW_SPACING:  The spacing between screwhole centers in both X and Y axes.
    SCREW_SIZE:     The diameter of the screws.
    SCREW_DEPTH:    The depth of the screwholes.
    SHAFT_DIAM:     The diameter of the motor shaft.
*/

// size represents the characteristic NEMA of the setup which will determine dimensions of the rest.
module idle_gusset_l_mount(size) {
    info = nema_motor_info(size);
    motor_width = info[0];
    plinth_height = info[1];

    translate([gusset_side / 2, 0, plinth_height])
        cube([motor_width + gusset_side, 0, plinth_height * 2], center = true);
    translate([motor_width / 2 + gusset_side + plinth_height, 0, motor_width / 2 + gusset_side / 2])
        rotate([0, 90, 0])
            cube([motor_width + gusset_side, 0, plinth_height * 2], center = true);
    difference() {
        translate([0, 0, 0]) {
            union() {
                translate([0, 0, plinth_height])
                    difference() {
                        translate([gusset_side / 2, 0, 0])
                        cube([
                            motor_width + gusset_side, 
                            ball_bearing_od * 2, 
                            plinth_height * 2
                        ], center=true);
                    }

                translate([
                    motor_width / 2 + gusset_side + plinth_height, 
                    0, 
                    motor_width / 2 + gusset_side / 2
                ])
                    rotate([0, 90, 0])
                        difference() {
                            cube([
                                motor_width + gusset_side, 
                                ball_bearing_od * 2, 
                                plinth_height * 2
                            ], center=true);
                        }
            }
            translate([(motor_width / 2 + gusset_side), -(ball_bearing_od), plinth_height * 2])
                rotate([90, 0, 180])
                    gusset(gusset_side, gusset_width);

            translate([(motor_width / 2 + gusset_side), ball_bearing_od - gusset_width, plinth_height * 2])
                rotate([90, 0, 180])
                    gusset(gusset_side, gusset_width);
        }

        translate([0, 0, 0])
            cylinder(d = ball_bearing_od, h = 100, center = true);
    }
}

module nema_gusset_l_mount(size) {
    info = nema_motor_info(size);
    motor_width = info[0];
    plinth_height = info[1];

    union() {
        translate([0, 0, plinth_height])
            difference() {
                translate([gusset_side / 2, 0, 0])
                cube([
                    motor_width + gusset_side, 
                    motor_width + 2 * gusset_width, 
                    plinth_height * 2
                ], center=true);
                nema_mount_mask(size=size);
            }

        translate([
            motor_width / 2 + gusset_side + plinth_height, 
            0, 
            motor_width / 2 + gusset_side / 2
        ])
            rotate([0, 90, 0])
                difference() {
                    cube([
                        motor_width + gusset_side, 
                        motor_width + 2 * gusset_width, 
                        plinth_height * 2
                    ], center=true);

                    translate([0, motor_width / 3, 0])
                        rotate([0, 0, 90])
                            tolerant_screw_hole(2 * motor_width / 3, mount_screw_d, plinth_height * 2 + 1);
                    translate([0, -motor_width / 3, 0])
                        rotate([0, 0, 90])
                            tolerant_screw_hole(2 * motor_width / 3, mount_screw_d, plinth_height * 2 + 1);
                }
    }

    translate([(motor_width / 2 + gusset_side), -(motor_width / 2 + gusset_width), plinth_height * 2])
        rotate([90, 0, 180])
            gusset(gusset_side, gusset_width);

    translate([(motor_width / 2 + gusset_side), motor_width / 2, plinth_height * 2])
        rotate([90, 0, 180])
            gusset(gusset_side, gusset_width);
}

module adjacent_idle_mounts(
    size,
    distances,
    positions
) {
    info = nema_motor_info(size);
    motor_width = info[0];
    plinth_height = info[1];

    nema_mount_width = motor_width + gusset_side;
    nema_mount_depth = motor_width + 2 * gusset_width;
    nema_mount_height = plinth_height * 2;
    nema_mount = [nema_mount_width, nema_mount_depth, nema_mount_height];

    idle_mount_width = nema_mount.x;
    idle_mount_depth = ball_bearing_od * 2; 
    idle_mount_height = nema_mount.z; 
    idle_mount = [idle_mount_width, idle_mount_depth, idle_mount_height];

    for(i = [0 : len(positions) - 1]) {
        center_distance = distances[i];
        gap_size = i == 0 ? center_distance - (idle_mount.y + nema_mount.y) / 2 : center_distance - idle_mount.y / 2;
        center_pos = i == 0 ? (nema_mount.y + gap_size) / 2 : positions[i] - center_distance / 2;

        gap_panel_width = nema_mount.x;
        gap_panel_depth = gap_size;
        gap_panel_height = nema_mount.z;
        gap_panel = [gap_panel_width, gap_panel_depth, gap_panel_height];

        translate([0, positions[i], 0])
            idle_gusset_l_mount(size);

        translate([
            gusset_side / 2, 
            center_pos, 
            nema_mount.z / 2
        ])
            cube(gap_panel, center = true);
        translate([
            (nema_mount.x + nema_mount.z + gusset_side) / 2, 
            center_pos,
            nema_mount.x / 2
        ])
            rotate([0, 90, 0])
                cube(gap_panel, center = true);
    }
}

module gear_test_stand(
    size,
    mod_val,
    z_list
) { 
    info = nema_motor_info(size);
    motor_width = info[0];
    plinth_height = info[1];

    nema_mount_width = motor_width + gusset_side;
    nema_mount_depth = motor_width + 2 * gusset_width;
    nema_mount_height = plinth_height * 2;
    nema_mount = [nema_mount_width, nema_mount_depth, nema_mount_height];

    idle_mount_width = nema_mount.x;
    idle_mount_depth = ball_bearing_od * 2; 
    idle_mount_height = nema_mount.z; 
    idle_mount = [idle_mount_width, idle_mount_depth, idle_mount_height];

    center_distance = ((z_list[0] + z_list[1]) * mod_val) / 2;
    gap_size = center_distance - (nema_mount.y + idle_mount.y) / 2;
    average_center = (nema_mount.y + gap_size) / 2;

    assert(len(z_list) >= 2, "Invalid test stand input");
    distance_list = [
        for(i = [0 : len(z_list) - 2]) (z_list[i] + z_list[i + 1]) * mod_val / 2
    ];
    position_list = cumsum(distance_list);

    nema_gusset_l_mount(size);
    adjacent_idle_mounts(
        size,
        distance_list,
        position_list
    );
}

module pulley_gear_test_stand(
    size,
    distances,
    positions
) {
    info = nema_motor_info(size);
    motor_width = info[0];
    plinth_height = info[1];

    nema_mount_width = motor_width + gusset_side;
    nema_mount_depth = motor_width + 2 * gusset_width;
    nema_mount_height = plinth_height * 2;
    nema_mount = [nema_mount_width, nema_mount_depth, nema_mount_height];

    idle_mount_width = nema_mount.x;
    idle_mount_depth = ball_bearing_od * 2; 
    idle_mount_height = nema_mount.z; 
    idle_mount = [idle_mount_width, idle_mount_depth, idle_mount_height];

    center_distance = distances[0];
    gap_size = center_distance - (nema_mount.y + idle_mount.y) / 2;
    average_center = (nema_mount.y + gap_size) / 2;

    gap_panel_width = nema_mount.x;
    gap_panel_depth = gap_size;
    gap_panel_height = nema_mount.z;
    gap_panel = [gap_panel_width, gap_panel_depth, gap_panel_height];

    nema_gusset_l_mount(size);
    adjacent_idle_mounts(
        size,
        distances,
        positions
    );
}

size = 17; 
mod = 2;
z = [24, 24];

// nema_gusset_l_mount(size);
gear_test_stand(
    size,
    mod,
    z
);

pulley_r = 6.4; 
pulley_ratio = 180;
pulley_belt_l = (pulley_ratio * PI / 180) * pulley_r;
distances = [(200 - 2 * pulley_belt_l) / 2, (24 + 24) * mod / 2];
positions = cumsum(distances);

!pulley_gear_test_stand(
    size,
    distances,
    positions
);