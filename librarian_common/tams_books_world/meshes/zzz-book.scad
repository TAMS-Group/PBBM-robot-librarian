/** book.scad - simple parametrizable book shape
 *  for robot manipulation experiments.
 *
 * Dimensions are in millimeters; scale by 0.001 for ROS.
 * Origin is at the center of the book.
 * 
 * (c) 2019, fnh, hendrich@informatik.uni-hamburg.de
 */
 
$fn = 50;
eps = 0.01;
demo = false; 
 

if (demo) { 
translate( [0,0,0] )  book(); // A4 size, soft cover
translate( [0,10,0] ) minski_society_of_mind();
translate( [0,20,0] ) reichelt_2018();
translate( [0,30,0] ) knuth_aocp();
translate( [0,40,0] ) hamburg_guide();
}


// translate( [0,0,0] ) minski_society_of_mind();
// translate( [0,0,0] ) knuth_aocp();
translate( [0,0,0] ) hamburg_guide();


// Knuth, AoCP volume 1
module knuth_aocp() {
book( height=24.0, width=16.5, thickness=8, // 3.7,
      cover_thickness=0.5,
      back_radius = 8,
      cover_color = [0.999,0.9,0.5] ,
      paper_color = [0.95,0.95,0.93]
     );
}


// Minsky, Society of Mind
module minski_society_of_mind() {
book( height=29.7, width=21.0, thickness=2.0,
      cover_thickness=0.3,
      cover_color = [0.999,0.9,0.5] ,
      paper_color = [0.95,0.95,0.93]
     );
}


// Reichelt catalogue 06.2018
module reichelt_2018() {
book( height=22.3, width=17.0, thickness=4.0,
      cover_thickness=0.2,
      cover_color = [0.7,0.7,0.7] ,
      paper_color = [0.95,0.95,0.93]
     );
}

 
// Hamburg guide IROS2015
module hamburg_guide() {
book( height=18.4, width=10.7, thickness=0.5,
      cover_thickness=0.1,
      cover_color = [0.0,0.4,0.7] ,
      paper_color = [0.95,0.95,0.93]
     );
}
 
 
module book(
  height = 29.7, 
  width = 21.0, 
  thickness = 2.0, 
  weight = 1.0,
  back_radius = 0, // or zero for flat
  cover_thickness = 0.3, // or zero for soft cover cover
  cover_color = [0.6, 0.3, 0.2],
  paper_color = [0.9, 0.9, 0.9],
)
{
  if (back_radius == 0) {
    color( cover_color ) 
      cube( size=[width,thickness,height], center=true );   
    delta = 0.01;
    color( paper_color ) 
      translate( [cover_thickness/4,0,0] )
      cube( size=[width-cover_thickness/2+delta,thickness-cover_thickness,height+delta], center=true );   
  } 
  else {
    difference() {
      
      intersection() {
        union() {
          translate( [-back_radius,0,0] ) 
            cube( size=[width, max( 2*back_radius, thickness ), height+eps], center=true );
          translate( [width/2-back_radius,0,0] )
            cylinder( r = back_radius, h=height+2*eps, center=true );
        }
        translate( [0,0,0] )
          cube( size=[width,thickness,height], center=true );   
      }
      
      alpha = atan2( thickness/2, back_radius );
      echo( alpha );
      dx = back_radius - back_radius*cos( alpha );
      // dx = 0.1;
      
      echo( dx );
      translate( [-width/2 - back_radius + dx,0,0] )
        cylinder( r = back_radius, h=height+2*eps, center=true );
    
   } // intersection
  } // difference
 
  
}