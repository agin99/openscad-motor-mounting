include <BOSL2/std.scad>
include <BOSL2/nema_steppers.scad>

$fn = 50;

// ========== CONSTANTS ========== //

gusset_width = 5;
gusset_side = 15;
mount_screw_d = 4;
ball_bearing_od = 14;
nema_14_shaft_l = 24; // \pm 1
nema_17_shaft_l = 24; // \pm 1
nema_23_shaft_l = 21; // \pm 0.5

// ========== LOGIC ========== //
function const_values() = [
    gusset_width,
    gusset_side,
    mount_screw_d,
    ball_bearing_od,
    nema_14_shaft_l,
    nema_17_shaft_l,
    nema_23_shaft_l
];

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

module nema_vertical_mount_base(
    size,
    screw_distance,
    screw_slit_l,
    screw_margin,
    ridge_depth = 3,
    ridge_height = 5,
    ridge_margin = 2,
    show_shaft = false
) {
    info = nema_motor_info(size);
    motor_width = info[0];
    plinth_height = info[1];

    motor_base_width = motor_width;
    motor_base_depth = motor_width;
    motor_base_height = plinth_height * 2;
    motor_base = [motor_base_width, motor_base_depth, motor_base_height];

    motor_ridge_clearance = 1; // 0.5mm per side
    mount_width = screw_distance + 2 * screw_margin;
    mount_depth = motor_base.y + ridge_depth + 2 * ridge_margin + motor_ridge_clearance;
    mount_height = motor_base.z;
    mount = [mount_width, mount_depth, mount_height];

    ridge_width = mount.x;
    ridge_depth = ridge_depth;
    ridge_height = ridge_height; 
    ridge = [ridge_width, ridge_depth, ridge_height];

    assert(mount_width > motor_width, "Mount width too low for motor size.");
    %translate([0, 0, mount.z])
        rotate([0, 180, 0])
            nema_stepper_motor(size=size);

    difference() {
        translate([0, 0, plinth_height])
            difference() {
                union() {
                    cube(mount, center=true);
                    translate([0, mount.y / 2 - ridge_margin, (ridge.z + mount.z) / 2])
                        cube(ridge, center=true);
                    translate([0, -(mount.y / 2 - ridge_margin), (ridge.z + mount.z) / 2])
                        cube(ridge, center=true);
                }
                nema_mount_mask(size=size);
            }

        translate([(mount.x / 2 - screw_margin), 0, 0])
            tolerant_screw_hole(mount.y - (2 * screw_margin + ridge_depth + 2 * ridge_margin) , mount_screw_d, 20);

        translate([-(mount.x / 2 - screw_margin), 0, 0])
            tolerant_screw_hole(mount.y - (2 * screw_margin + ridge_depth + 2 * ridge_margin), mount_screw_d, 20);
    }
}

module nema_vertical_mount_support_mask(
    size,
    col_height,
    screw_distance,
    screw_slit_l,
    screw_margin,
    ridge_depth = 3,
    ridge_height = 5,
    ridge_margin = 2,
    show_shaft = false
) {
    info = nema_motor_info(size);
    motor_width = info[0];
    plinth_height = info[1];

    motor_base_width = motor_width;
    motor_base_depth = motor_width;
    motor_base_height = plinth_height * 2;
    motor_base = [motor_base_width, motor_base_depth, motor_base_height];

    motor_ridge_clearance = 1; // 0.5mm per side
    mount_width = screw_distance + 2 * screw_margin;
    mount_depth = motor_base.y + ridge_depth + 2 * ridge_margin + motor_ridge_clearance;
    mount_height = motor_base.z;
    mount = [mount_width, mount_depth, mount_height];

    ridge_width = mount.x;
    ridge_depth = ridge_depth;
    ridge_height = ridge_height; 
    ridge = [ridge_width, ridge_depth, ridge_height];
    
    support_col_width = 2 * mount_screw_d;
    support_col_depth = mount.y - (2 * screw_margin + ridge_depth + 2 * ridge_margin); 
    support_col_height = col_height;
    support_col = [support_col_width, support_col_depth, support_col_height];

    union() {
        translate([0, 0, -(support_col.z + mount.z / 2)])
        cube(mount, center=true);
        translate([(mount.x / 2 - screw_margin), 0, -(support_col.z / 2)])
            cube(support_col, center = true);

        translate([-(mount.x / 2 - screw_margin), 0, -(support_col.z / 2)])
            cube(support_col, center = true);
    }
}

module gusset_l_mount(plate) {
    union() {
        //bore plate
        translate([0, 0, plate.z / 2])
            cube(plate, center=true);

        // base plate
        translate([(plate.x + plate.z) / 2, 0, plate.x / 2])
            rotate([0, 90, 0])
                cube(plate, center=true);
    }
    translate([plate.x / 2, -(plate.y / 2), plate.z]) {
        rotate([90, 0, 180])
            gusset(gusset_side, gusset_width);

        translate([0, plate.y - gusset_width, 0])
            rotate([90, 0, 180])
                gusset(gusset_side, gusset_width);
    }
}

module base_gusset_l_mount(plate) {
    difference() {
        translate([gusset_side / 2, 0, 0]) 
            gusset_l_mount(plate);
        
        children(); 
    }
}

module idle_gusset_l_mount(size) {
    info = nema_motor_info(size);
    motor_width = info[0];
    plinth_height = info[1];

    //base plate
    plate_width = motor_width + gusset_side;
    plate_depth = ball_bearing_od * 2;
    plate_height = plinth_height * 2;
    plate = [plate_width, plate_depth, plate_height];

    base_gusset_l_mount(plate)
        cylinder(d = ball_bearing_od, h = 100, center = true);
}

module nema_gusset_l_mount(size) {
    info = nema_motor_info(size);
    motor_width = info[0];
    plinth_height = info[1];

    plate_width = motor_width + gusset_side;
    plate_depth = motor_width + 2 * gusset_width;
    plate_height = plinth_height * 2;
    plate = [plate_width, plate_depth, plate_height];

    base_gusset_l_mount(plate) {
        translate([0, 0, plate.z / 2])
            nema_mount_mask(size=size);

        translate([(plate.x + plate.z + gusset_side) / 2, 0, plate.x / 2]) {
            translate([0, motor_width / 3, 0])
                rotate([90, 0, 90])
                    tolerant_screw_hole(2 * motor_width / 3, mount_screw_d, plinth_height * 2 + 1);
            translate([0, -motor_width / 3, 0])
                rotate([90, 0, 90])
                    tolerant_screw_hole(2 * motor_width / 3, mount_screw_d, plinth_height * 2 + 1);
        }
    }
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

// ========== BUILD ========== //

module gear_test_stand(
    size,
    mod_val,
    z_list
) {
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
    nema_gusset_l_mount(size);
    adjacent_idle_mounts(
        size,
        distances,
        positions
    );
}

// ========== ASSEMBLY ========== //

size = 17; 
mod = 2;
z = [24, 24];

*idle_gusset_l_mount(size);

*nema_gusset_l_mount(size);
*gear_test_stand(
    size,
    mod,
    z
);

pulley_r = 6.4; 
pulley_ratio = 180;
pulley_belt_l = (pulley_ratio * PI / 180) * pulley_r;
distances = [(200 - 2 * pulley_belt_l) / 2, (24 + 24) * mod / 2];
positions = cumsum(distances);

*pulley_gear_test_stand(
    size,
    distances,
    positions
);

nema_vertical_mount_base(size, 80, 20, mount_screw_d, show_shaft = false);

nema_vertical_mount_support_mask(
    size,
    col_height = 30,
    screw_distance = 80,
    screw_slit_l = 20,
    screw_margin = mount_screw_d
);