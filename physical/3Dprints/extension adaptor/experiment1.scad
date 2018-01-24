union(){
cube([14,2,30]);

color([1,0,0]);
translate([14,0,0]){
cube([14,5,0]);
}

translate([0,2,0]){
color(["red"]) cube([14,3,2]);
}

}