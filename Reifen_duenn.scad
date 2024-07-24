
$fn = 300; // Anzahl der Facetten für hohe Rundungsqualität

// Radius und Höhe des Zylinders
radius = 198;
height = 13;

// Radius der Rundung
fillet_radius = 6;
difference(){
translate([0,0,7])
minkowski() {
    $fn = 300; 
    cylinder(h = height - 2 * fillet_radius, r = radius);
    sphere(r = fillet_radius);
}
translate([0,0,0])
cylinder(15,199.5,199.5);
    $fn=120;

}