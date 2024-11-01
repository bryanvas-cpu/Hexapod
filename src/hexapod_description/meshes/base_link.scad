% scale(1000) import("base_link.STL");

// Append pure shapes (cube, cylinder and sphere), e.g:

// cylinder(r=10, h=10, center=true);
delta = 60;
for(i =[0:5])
{
    phi = i*delta;
    translate([120*cos(phi+30),120*sin(phi+30),20])
    rotate([0,0,phi-60])
        cube([25, 50, 30], center=true);
}


for(i =[0:5])
{
    phi = i*delta;
    translate([90*cos(phi),90*sin(phi),20])
    rotate([0,0,phi])
        cube([5, 70, 30], center=true);
}

translate([-30,-10,59])
cube([54,44,24],center=true);

translate([0,0,40])
cylinder(r=93, h=8, center=true);

translate([0,0,4])
cylinder(r=93, h=8, center=true);