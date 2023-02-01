difference(){
    cube(size=[0.765+0.016*2 ,0.36+0.016, 1.25], center = false);
    
    union(){
        translate([0.016,0.016,1.235-0.287]){
            cube([0.765,0.5, 0.287], center=false);
            };
        translate([0.016,0.016,1.235-0.287-0.335-0.016]){
            cube([0.765,0.5, 0.335], center=false);
            };
    
    };
};
    
