# make-ply.sh - create .ply meshes for BLORT object detection
# we need ROS running for this, as we take the size from the
# books.yaml file.
# Note: openscad files and output .ply files are in /tmp;
# copy from there to where you need the files.
#
echo "make-ply.sh"

rosparam load ../../config/medicine.yaml

# current list of books...
#
for i in "aronal" "ass" "bestview" "cetirizin" "cremeseife" "doppelherz" "kokett" "lipton" "lopedium" "pflaster" "sponge"
do
  echo medicine-$i.ply
  echo "cube( center=true, size=" > /tmp/$i.scad
  rosparam get "/medicine/$i/size" >> /tmp/$i.scad
  echo ");" >> /tmp/$i.scad
  openscad -o /tmp/$i.stl /tmp/$i.scad
  assimp export /tmp/$i.stl /tmp/$i.ply -f ply
done



  

