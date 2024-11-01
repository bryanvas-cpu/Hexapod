% scale(1000) import("t2.STL");

// Append pure shapes (cube, cylinder and sphere), e.g:
// cube([10, 10, 10], center=true);
// cylinder(r=10, h=10, center=true);
// sphere(10);
translate([0,0,-1.5])
cylinder(r=17, h=3, center=true);

translate([0,0,-43.5])
cylinder(r=17, h=3, center=true);

rotate([0,0,90])
rotate([0,90,15])
translate([22.5,12,-16])
cube([45, 34, 3], center=true);

rotate([0,0,90])
translate([10,40,-3])
rotate([0,5,0])
translate([82,0,0])

cube([140,2,2], center = true);

rotate([0,0,90])
translate([10,40,-42])
rotate([0,-5,0])
translate([82,0,0])

cube([140,2,2], center = true);

rotate([0,0,90])
translate([10,75,-35])
rotate([0,-3.5,-10])
translate([82,0,0])

cube([140,2,2], center = true);

rotate([0,0,90])
translate([10,75,-10])
rotate([0,3.5,-10])
translate([82,0,0])

cube([140,2,2], center = true);