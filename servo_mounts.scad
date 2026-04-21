use <../openscad-motor-coupling/servo_mg90s.scad>;
use <../openscad-motor-coupling/clamping_hub.scad>;

$fn = 100;

// ========== CONSTANTS ========== //

/* Micro Servo MG90S Dimensions:
    > total_height: 33mm

    > base_width:   23
    > base_depth:   12.5mm 
    > base_height:  23

    > wing_width:   33
    > wing_depth:   12.5
    > wing_height:  3

    > bottom_to_wing:   18
    > wing_screw_dist:  27.7 

    > plinth_width:     15 
    > plinth_depth:     12.5
    > plinth_height:    7

    > spline_height:    3
    > spline_diam:      5
*/

servo_base_width =    23; //Measured
servo_base_depth =    12.5;
servo_base_height =   23; //Measured
servo_base = [servo_base_width, servo_base_depth, servo_base_height];

wing_buff =     5;
function buff(size) = size + wing_buff;
wing_width =    buff(33);
wing_depth =    buff(12.5);
wing_height =   buff(3);
wing = [wing_width, wing_depth, wing_height];

wing_screw_dist =       27.7;
base_bottom_to_wing =   18;

screw_d =   2;
screw_l =   20; 
screw = [screw_d, screw_l];

plinth_width =  15;
plinth_depth =  12.5;
plinth_height = 10;
plinth = [plinth_width, plinth_depth, plinth_height];

support_base_width = wing.x;
support_base_depth = servo_base.z + 5; 
support_base_height = 5;
support_base = [support_base_width, support_base_depth, support_base_height];

support_col_width =   (wing.x - servo_base.x) / 2;
support_col_depth =   wing.z;
support_col_height =  0;
support_col = [support_col_width, support_col_depth, support_col_height];

mount_screw_d =     3;
mount_screw_l =     20;
mount_screw_dist =  6;

// ========== LOGIC ========== //

// ========== STRUCTURES ========== //
module mirror_copy(vector) {
    children();
    mirror(vector) children();
}

module tolerant_screw_hole(separation, d, l) {
    hull() {
        mirror_copy([0, 1, 0])
            translate([0, -separation / 2, 0])
                cylinder(d = d, h = l, center = true);
    }
}

module tolerant_nut_trap(separation, thickness, d) {
    hull() {
        mirror_copy([0, 0, 1])
            translate([0, 0, -separation / 2])
                rotate([90, 0, 0])
                    linear_extrude(thickness)
                        nut(d);
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

module mg90s_mount_base() {
    nut_thickness = 1;
    nut_dist =      2;

    difference() {
        cube([wing.x, wing.z, servo_base.y], center = true);
        rotate([90, 0, 0])
            mg90s_micro_servo_mask();
        cube([servo_base.x, wing.z + 1, servo_base.y], center = true);

        mirror_copy([1, 0, 0]) 
            translate([-wing_screw_dist / 2, (wing.z) / 2, 0])
                tolerant_nut_trap(nut_dist, nut_thickness, screw_d);
    }

    translate([0, (support_base.y - support_col.y) / 2, -(servo_base.y + support_base.z) / 2 - support_col.z])
        difference() {
            union() {
                cube(support_base, center = true);
                mirror_copy([1, 0, 0]) 
                    translate([
                        -(support_base.x - support_col.x) / 2, 
                        -(support_base.y - support_col.y) / 2, 
                        (support_base.z + support_col.z) / 2
                    ])
                        cube(support_col, center = true);
            }

            mirror_copy([1, 0, 0]) 
                translate([wing.x / 2 - mount_screw_d, 2, 0]) 
                    tolerant_screw_hole(mount_screw_dist, mount_screw_d, mount_screw_l);
        }
}

// ========== BUILD ========== //

module servo_test_stand(
    mod_val,
    z1, 
    z2
) {
    gusset_side_l = 15;
    gusset_width =  5;

    center_distance = ((z1 + z2) * mod_val) / 2; // Measured from servo shaft
    edge_to_servo_shaft = support_base.z + servo_base.y / 2;
    servo_shaft_height = servo_base.x / 2 - plinth.y / 2;

    xz_plate_width = support_base.y;
    xz_plate_depth = support_base.z;
    xz_plate_height = support_base.x + gusset_side_l;
    xz_plate = [xz_plate_width, xz_plate_depth, xz_plate_height];

    xy_plate_width = edge_to_servo_shaft + center_distance + xz_plate.x / 2; // Increase for shaft board once determined.
    xy_plate_depth = support_base.y;
    xy_plate_height = 5;
    xy_plate = [xy_plate_width, xy_plate_depth, xy_plate_height];

    yz_plate_width = support_base.z;
    yz_plate_depth = support_base.y;
    yz_plate_height = support_base.x + gusset_side_l + xy_plate.z; 
    yz_plate = [yz_plate_width, yz_plate_depth, yz_plate_height];

    translate([0, base_bottom_to_wing + (wing.z - wing_buff) / 2, 0])
        rotate([90, 0, 0])
            rotate([0, 0, 90])
                color("red", 0.2)
                    %mg90s_micro_servo();
    rotate([0, 90, 0]) {
        %mg90s_mount_base();
    }

    // base (xy) plate
    translate([
        xy_plate.x / 2 - edge_to_servo_shaft, 
        (support_base.y - wing.z) / 2, 
        -(support_base.x + xy_plate.z) / 2 - gusset_side_l
    ])
        cube(xy_plate, center = true);

    // servo (yz) plate
    difference() {
        translate([
            -(edge_to_servo_shaft + yz_plate.x / 2),
            (support_base.y - wing.z) / 2,
            -(yz_plate.z - support_base.x) / 2
        ])
            cube(yz_plate, center = true);

        translate([-mount_screw_l / 2, (support_base.y - support_col.y) / 2, 0])
            rotate([0, 90, 0]) {
                mirror_copy([1, 0, 0])
                    translate([wing.x / 2 - mount_screw_d, 2, 0]) 
                        cylinder(d = mount_screw_d, h = mount_screw_l, center = true);
            }
    }
    translate([
        -edge_to_servo_shaft, 
        yz_plate.y - wing.z / 2, 
        -yz_plate.z + support_base.x / 2 + xy_plate.z
    ])
        rotate([90, 0, 0])
            gusset(gusset_side_l, gusset_width);
    translate([
        -edge_to_servo_shaft, 
        gusset_width - wing.z / 2, 
        -yz_plate.z + support_base.x / 2 + xy_plate.z
    ])
        rotate([90, 0, 0])
            gusset(gusset_side_l, gusset_width);

    // shaft (xz) plate
    translate([
        center_distance,
        xz_plate.y / 2 - wing.z / 2, 
        0,
    ])
        difference() {
            translate([0, 0, -(xz_plate.z - support_base.x) / 2])
                cube(xz_plate, center = true);
            translate([0, 0, servo_shaft_height])
                rotate([90, 0, 0])
                    cylinder(d = 14, h = 20, center = true);
        }
    
    translate([
        center_distance + gusset_width - xz_plate.x / 2, 
        0, 
        -yz_plate.z + support_base.x / 2 + xy_plate.z
    ])
        rotate([0, -90, 0])
            gusset(gusset_side_l, gusset_width);
    translate([
        center_distance + xz_plate.x / 2, 
        0, 
        -yz_plate.z + support_base.x / 2 + xy_plate.z
    ])
        rotate([0, -90, 0])
            gusset(gusset_side_l, gusset_width);
}

// ========== ASSEMBLY ========== //
*mg90s_mount_base();
!servo_test_stand(
    2,
    24,
    24
);