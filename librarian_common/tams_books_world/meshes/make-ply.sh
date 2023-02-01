# make-ply.sh - create .ply meshes for BLORT object detection
# we need ROS running for this, as we take the size from the
# books.yaml file.
# Note: openscad files and output .ply files are in /tmp;
# copy from there to where you need the files.
#
echo "make-ply.sh"

rosparam load ../config/books.yaml

# current list of books...
#
for i in "bobh" "dyson" "goedel" "hamburg" "haykin" "javaff" "jekyll" "knuth" "knuth-40" "knuth-41" "knuth-42" "lagemann" "lakoff" "lskm" "mead-conway" "memory" "minsky" "moeller" "pdp1" "pdp2" "rojas" "schmidt" "selfish" "watson" "wiering" "winograd" "zuse"
do
  echo book-$i.ply
  echo "cube( center=true, size=" > /tmp/$i.scad
  rosparam get "/books/book_$i/size" >> /tmp/$i.scad
  echo ");" >> /tmp/$i.scad
  openscad -o /tmp/$i.stl /tmp/$i.scad
  assimp export /tmp/$i.stl /tmp/$i.ply -f ply
done



