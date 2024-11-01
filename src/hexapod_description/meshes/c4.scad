% scale(1000) import("c4.STL");

// Append pure shapes (cube, cylinder and sphere), e.g:
// cube([10, 10, 10], center=true);
// cylinder(r=10, h=10, center=true);
// sphere(10);
theta = 30;
alpha = 0;
length = 21;

translate([0,0,1.25])
cylinder(r=20, h=2.5, center=true);

translate([0,0,48.5])
cylinder(r=30, h=5, center=true);

translate([length*cos(theta+alpha),length*(sin(theta+alpha)),24])
rotate([0,0,theta+alpha])
cube([10, 1, 45], center=true);

translate([length*-cos(theta-alpha),length*(sin(theta-alpha)),24])
rotate([0,0,-theta+alpha])
cube([10, 1, 45], center=true);