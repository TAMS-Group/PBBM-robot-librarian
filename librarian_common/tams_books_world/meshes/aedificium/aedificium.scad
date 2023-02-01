/* Openscad model of Umberto Eco's Aedificium library
 * 
 * based on https://upload.wikimedia.org/wikipedia/commons/thumb/0/0f/Labyrinthus_Aedificium.svg/1024px-Labyrinthus_Aedificium.svg.png
 * based on https://kg3.de/rose/aedificium.gif
 *
 * 2021.12.10 - implement
 *
 * (C) 2021 fnh, norman.hendrich@uni-hamburg.de
 */
 
eps = 1;
 
t2 = 700.0; // outer wall thickness
t1 = 350.0; // inner wall thickness
 
h2 = 3000.0; // total room height
h1 = 1100.0; // bottom height of windows
ww =  700.0; // window width
wh =  900.0; // window height
 
r1 = 5000;   // radius of inner court

enable_labels = 1;
enable_shelves = 0;
make_map = 0;
 

if (make_map) {
  intersection() {
    aedificium();
    cube( [37000,37000,2500], center=true );
  }
}
else aedificium();

// translate( [0,0,0] ) door();
// translate( [0,0,0] ) spitzbogentest();
// ground_label( "HUGO" );
// bookshelf();


module spitzbogentest() {
    intersection() {
      translate( [0,-40,0] ) 
        cylinder( d=ww, h=t2, $fn=50, center=true );   
      translate( [0, 40,0] ) 
        cylinder( d=ww, h=t2, $fn=50, center=true );   
    }
}


/**
 * a wooden bookshelf with origin centered at its bottom.
 */
module bookshelf( sx=350, sy=600, sz=2000, tt=20, hb=[350,650,1000,1350,1700], rear=true )
{
  color( "burlywood" ) {
  translate( [0,0,100/2] ) cube( [sx,sy,100], center=true );
  translate( [0,0,sz-tt/2] ) cube( [sx,sy,tt], center=true );
    
  translate( [0,-sy/2+tt/2,sz/2] ) cube( [sx,tt,sz], center=true );
  translate( [0,+sy/2-tt/2,sz/2] ) cube( [sx,tt,sz], center=true );
    
  for( sz = hb ) {
    translate( [0,0,sz] ) cube( [sx,sy,tt], center=true ); 
  }  
  
  if (rear) {
    translate( [sx/2,0,sz/2] ) cube( [tt,sy,sz], center=true );   
  }
  } // color
}


module door( sx=t2, sy=1000, sz=1900 ) {
  translate( [0,0,sz/2-sy/4+eps] )
    cube( [sx,sy,sz-sy/2], center=true );
  intersection() {
    dd = 100;
    translate( [0,dd,sz-sy/2] ) rotate( [0,90,0] )
      cylinder( d=sy+2*dd, h=sx+eps, $fn=50, center=true );     
    translate( [0,-dd,sz-sy/2] ) rotate( [0,90,0] )
      cylinder( d=sy+2*dd, h=sx+eps, $fn=50, center=true );     
  }
}
 

module wall_with_window( sx=t2, sy=2000, sz=h2, ww=ww ) 
{
  difference() {
    translate( [0,0,h2/2] ) cube( [sx,sy,sz], center=true );   
    translate( [0,0,h1+wh/2] ) cube( [sx+eps,ww,wh], center=true );
    intersection() {
      dd = 90;
      translate( [0,-dd,h1+wh] ) rotate( [0,90,0] ) cylinder( d=ww+2*dd, h=sx+eps, $fn=50, center=true );
      translate( [0, dd,h1+wh] ) rotate( [0,90,0] ) cylinder( d=ww+2*dd, h=sx+eps, $fn=50, center=true );
    }
  }
}


module middle_wall( sx=t1, sy=3400, sz=h2, door=1 ) 
{
  difference() {
    translate( [0,0,h2/2] ) cube( [sx,sy,sz], center=true );   
    if (door == 1) {
      translate( [0,0,-2*eps] ) door( sx=t2+eps, sy=1000, sz=1800 );
    }
  }
  if (door == 2) { // entrance to finis africae
    color( [0,1,1,1] )  
      translate( [0,0,-2*eps] ) door( t1+100, sy=1000, sz=1800 );
  }
}
 
 
module aedificium()
{
    
  // inner walls with windows to the inner court
  y1 = 2215;
  for( i = [0:7] ) {
    color( [0.85, 0.8, 0.8] ) 
    rotate( [0,0,i*45] ) {
      translate( [r1,-y1/2,0] ) wall_with_window( sy=y1 );
      translate( [r1, y1/2,0] ) wall_with_window( sy=y1 );
    }
  }     

  // middle "tangential" walls with doors according to labyrinth
  mdoors = [1,1,1,1,1,1,1,0, 0,0,1,1,1,1,1,1];
  r2 = r1 + 3000;
  y2 = 3400;
  for( i = [0:7] ) {
    color( [0.85, 0.8, 0.8] ) 
    rotate( [0,0,i*45] ) {
      translate( [r2, -y2/2, 0] ) middle_wall( door=mdoors[2*i] );
      translate( [r2, +y2/2, 0] ) middle_wall( door=mdoors[2*i+1] );
    }
  }     
  
  // middle "radial" walls with doors according to labyrinth
  rdoors = [1,0,1,1, 1,1,1,1, 1,0,0,1, 0,1,1,1];
  rlabel = ["C", "E", "G", "A", "I", "A", "M", "O", "R", "Y", "T", "P", "E", "A", "E", "A"];

  sdoors = [1,0,1,0, 1,1,1,1, 1,1,1,0, 1,0,1,1];
  slabel = ["A", "R", "M", "L", "L", "I", "N", "A", "P", "S", "U", "Y", "G", "D", "U", "I"];

  for( i=[0:15] ) {
    rotate( [0,0,i*22.5] ) translate( [(r1+r2)/2+450*(i%2), 0, eps] ) rotate( [0,0,90] ) 
      middle_wall( sy = 3000, door=rdoors[i] );
    rotate( [0,0,i*22.5+11.25] ) translate( [(r1+r2)/2+450, 0, eps] ) rotate( [0,0,-i*22.5-11.25] )   
      ground_label( rlabel[i] );
      
    rotate( [0,0,i*22.5] ) translate( [(r1+r2)/2+3300+650*(i%2), 0, -eps] ) rotate( [0,0,90] ) 
      middle_wall( sy = 4000, door=sdoors[i] );
    rotate( [0,0,i*22.5+11.25] ) translate( [(r1+r2)/2+3300, 0, eps] ) rotate( [0,0,-i*22.5-11.25] )   
      ground_label( slabel[i] );
  }     
 
  // outer "between-towers" walls with windows
  r3 = r1 + 6500;
  y3 = 4600;
  for( theta = [0, 90, 180, 270] ) {
    color( [0.85, 0.8, 0.8] ) 
    rotate( [0,0,theta] ) {
      translate( [r3,-y3/2,0] ) wall_with_window( sy=y3, ww=1000 );
      translate( [r3, y3/2,0] ) wall_with_window( sy=y3, ww=1000 );
    }
  }     
 
  // outer walls of the towers
  r4 = r1 + r2 + 2500; // center of tower
  r5 = 12500;
  
  // tower i, inner door k 
  ikdoors = [[0,0,1,1,1,1,1],[0,0,0,0,1,1,0],[0,0,0,0,2,0,0],[1,0,0,1,1,1,0]];
  ildoors = [[1,1,0,1,0,1,1],[1,1,1,1,0,1,1],[1,1,0,1,1,1,1],[1,1,0,1,0,1,1]];
  ilabel  = ["A", "A", "*", "A"];
  iklabel = [["G", "L", "I", " ", " ", "I", "N"],
             ["B", "E", "R", " ", " ", "H", "I"],
             ["O", "E", "L", " ", " ", "E", "N"],
             [" ", "O", "F", " ", " ", "S", "N"],
            ];
  
  for( i = [0:3] ) {
    rotate( [0,0,i*90+45] ) translate( [r4,0,0] ) {
        
      rotate( [0,0,-i*90-45] ) ground_label( ilabel[i] );  
      // 7-polygons to layout the towers before creating the walls
      // color( "black" )
      // translate( [0,0,eps] ) rotate( [0,0,180/7] ) 
      //   cylinder( d=r5, h=100, $fn=7, center=false );
      // color( "gray" )
      // rotate( [0,0,180/7] ) 
      //   cylinder( d=6000, h=600, $fn=7, center=false );
        
      for( j=[-2,-1,0,1,2] ) { // outer tower walls with windows
        color( [0.85, 0.8, 0.8] ) 
        rotate( [0,0,j*360/7] ) translate( [r5/2-650, 0,0] ) 
          wall_with_window( sy=5730, ww=1000 );
      }   

      for( k=[0:6] ) { // inner tower room walls with doors
        rotate( [0,0,k*360/7] ) translate( [2800, 0,0] ) 
           middle_wall( sy=3350, door=ikdoors[i][k] );
      }

      for( l=[0:6] ) { // radial walls in the towers with doors
        rotate( [0,0,l*360/7+180/7] ) 
          if (l == 3) {
            translate( [3500, 0,0] ) rotate( [0,0,90] ) 
              middle_wall( sy=600, door=0 );
          }
          else {
           translate( [4500, 0,0] ) rotate( [0,0,90] ) 
             middle_wall( sy=2800, door=ildoors[i][l] );
          }
      }

      for( l=[0:6] ) { // labels for the outer rooms in the towers
        rotate( [0,0,l*360/7] ) 
          translate( [4000,0,0] ) 
            rotate( [0,0,-l*360/7-i*90-45] )
              ground_label( iklabel[i][l] );
      }
    } // end translate r4
  } // end for i
  
  // bookshelves: 
  
  // shelves on middle "radial" walls with doors according to labyrinth
if (enable_shelves) {
  for( i=[0:15] ) {
    sx = t1; sy = 900; sz = 1800;
    if (i % 2) {
      if (rdoors[i] ) {
        rotate( [0,0,i*22.5] ) translate( [(r1+r2)/2+500, 0, eps] ) 
          rotate( [0,0,90] ) translate( [-sx, sy,0] ) bookshelf( sx=sx, sy=700, sz=sz );
        rotate( [0,0,i*22.5] ) translate( [(r1+r2)/2+400, 0, eps] ) 
          rotate( [0,0,-90] ) translate( [-sx, sy,0] ) bookshelf( sx=sx, sy=700, sz=sz );

        rotate( [0,0,i*22.5] ) translate( [(r1+r2)/2+400, 0, eps] ) 
          rotate( [0,0,90] ) translate( [-sx, -sy,0] ) bookshelf( sx=sx, sy=700, sz=sz );
        rotate( [0,0,i*22.5] ) translate( [(r1+r2)/2+500, 0, eps] ) 
          rotate( [0,0,-90] ) translate( [-sx, -sy,0] ) bookshelf( sx=sx, sy=700, sz=sz );
      }       
      else {
        rotate( [0,0,i*22.5] ) translate( [(r1+r2)/2-400, 0, eps] ) 
          rotate( [0,0,90] ) translate( [-sx, -sy,0] ) bookshelf( sx=sx, sy=2400, sz=sz );
        rotate( [0,0,i*22.5] ) translate( [(r1+r2)/2+1400, 0, eps] ) 
          rotate( [0,0,-90] ) translate( [-sx, -sy,0] ) bookshelf( sx=sx, sy=2400, sz=sz );
      }
    }
    else {
      if (rdoors[i] ) {
        rotate( [0,0,i*22.5] ) translate( [(r1+r2)/2+100, 0, eps] ) 
          rotate( [0,0,90] ) translate( [-sx, sy,0] ) bookshelf( sx=sx, sy=600, sz=sz );
        rotate( [0,0,i*22.5] ) translate( [(r1+r2)/2-1700, 0, eps] ) 
          rotate( [0,0,-90] ) translate( [-sx, sy,0] ) bookshelf( sx=sx, sy=600, sz=sz );

        rotate( [0,0,i*22.5] ) translate( [(r1+r2)/2+1800, 0, eps] ) 
          rotate( [0,0,90] ) translate( [-sx, sy,0] ) bookshelf( sx=sx, sy=800, sz=sz );
        rotate( [0,0,i*22.5] ) translate( [(r1+r2)/2+0, 0, eps] ) 
          rotate( [0,0,-90] ) translate( [-sx, sy,0] ) bookshelf( sx=sx, sy=800, sz=sz );
      }
      else {  
        rotate( [0,0,i*22.5] ) translate( [(r1+r2)/2-800, 0, eps] ) 
          rotate( [0,0,90] ) translate( [-sx, -sy,0] ) bookshelf( sx=sx, sy=2400, sz=sz );
        rotate( [0,0,i*22.5] ) translate( [(r1+r2)/2+1000, 0, eps] ) 
          rotate( [0,0,-90] ) translate( [-sx, -sy,0] ) bookshelf( sx=sx, sy=2400, sz=sz );
      }
    }
//
//      if (rdoors[i] ) {
//        rotate( [0,0,i*22.5] ) translate( [(r1+r2)/2+100+450*(i%2), 0, eps] ) {
//        rotate( [0,0,90] ) translate( [-sx, sy,0] ) bookshelf( sx=sx, sy=600, sz=sz );
//        #rotate( [0,0,90] ) translate( [-sx,-sy,0] ) bookshelf( sx=sx, sy=600, sz=sz );
//    }
//    rotate( [0,0,i*22.5] ) translate( [(r1+r2)/2+100+450*(i%2), 0, eps] ) {
//      if (rdoors[i] ) {
//        translate( [-sy, sx,0] ) rotate( [0,0,-90] ) bookshelf( sx=sx, sy=600, sz=sz );
//        translate( [+sy, sx,0] ) rotate( [0,0,-90] ) bookshelf( sx=sx, sy=600, sz=sz );
//      }
//      else {
//        rotate( [0,0,90] ) translate( [-sx, 0,0] ) bookshelf( sx=sx, sy=2400, sz=sz );
//        translate( [0,sx,0] ) rotate( [0,0,-90] ) bookshelf( sx=sx, sy=2400, sz=sz );
//      }
//    }
  }

  // shelves on outer "radial" walls with doors according to labyrinth
  for( i=[0:15] ) {
    sx = t1+50; sy = 1400; sz = 1800;
    rr = (r1+r2)/2+3300+650*(i%2);
    syy = [920, 900, 1800, 1500];
    rotate( [0,0,i*22.5] ) translate( [rr, 0, eps] ) {
      sy = syy[i%4]; 
      dy = sy/2 + 400; // half door-width + bookshelf width
      if (sdoors[i] ) {
        rotate( [0,0,90] ) translate( [-sx, 900,0] ) bookshelf( sx=sx, sy=1000, sz=sz );
        rotate( [0,0,90] ) translate( [-sx,-900,0] ) bookshelf( sx=sx, sy=700, sz=sz );
        translate( [-900, sx,0] ) rotate( [0,0,-90] ) bookshelf( sx=sx, sy=1000, sz=sz );
        translate( [+900, sx,0] ) rotate( [0,0,-90] ) bookshelf( sx=sx, sy=700, sz=sz );
      }
      else {
        // rotate( [0,0,90] ) translate( [-sx, -dy-1000,0] ) bookshelf( sx=sx, sy=1000, sz=sz );
        // rotate( [0,0,90] ) translate( [-sx, -dy,0] ) bookshelf( sx=sx, sy=1000, sz=sz );
        
          rotate( [0,0,90] ) translate( [-sx, -dy+1000,0] ) bookshelf( sx=sx, sy=2400, sz=sz );
        translate( [0,sx,0] ) rotate( [0,0,-90] ) bookshelf( sx=sx, sy=2400, sz=sz );
      }
    }

} // if enable_shelves    
  }
}




module ground_label( string, letter_size=1000, halign="center", valign="center" ) {
  font = "Liberation Sans";
  // Use linear_extrude() to make the letters 3D objects as they
  // are only 2D shapes when only using text()
  if (enable_labels) {
    color( "gray" ) 
    translate( [0,0,-20] ) 
    rotate( [0,0,-45] ) 
    linear_extrude(height = 10) {
      text( string, size = letter_size, font = font, halign = halign, valign = valign, $fn = 16);
    }
  }
}

