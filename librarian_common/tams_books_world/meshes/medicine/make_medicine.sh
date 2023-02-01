#!/bin/tcsh
# make-medicine.sh: copy template Collada (cube.dae) and apply
# sed magic to replace the dummy textures with actual scanned
# textures.
echo "make-medicine.sh"


# Philipp's Layout is: 
# 1=back@180 2=cover@0 3=top@90 4=bottom@270 5=spine@-90 6=front-edge@90
#
foreach i ("aronal" "ass" "bestview" "cetirizin" "cremeseife" "doppelherz" "kokett" "lipton" "lopedium" "pflaster" "sponge")
  echo medicine-$i.dae
  cp cube.dae /tmp/
  jpegtopnm $i-back.jpg | pnmflip -r180 | pnmtojpeg > $i-back-180.jpg
  sed -i s/1.png/$i-back-180.jpg/ /tmp/cube.dae
  #
  sed -i s/2.png/$i-cover.jpg/ /tmp/cube.dae
  # 
  # jpegtopnm $i-top.jpg | pnmflip -r90 | pnmtojpeg > $i-top-90.jpg
  sed -i s/3.png/$i-top.jpg/ /tmp/cube.dae
  #
  jpegtopnm $i-bottom.jpg | pnmflip -r180 | pnmtojpeg > $i-bottom-180.jpg
  sed -i s/4.png/$i-bottom-180.jpg/ /tmp/cube.dae
  #
  jpegtopnm $i-spine.jpg | pnmflip -r270 | pnmtojpeg > $i-spine-270.jpg
  sed -i s/5.png/$i-spine-270.jpg/ /tmp/cube.dae
  #
  jpegtopnm $i-front-edge.jpg | pnmflip -r90 | pnmtojpeg > $i-front-edge-90.jpg
  sed -i s/6.png/$i-front-edge-90.jpg/ /tmp/cube.dae
  cp /tmp/cube.dae box-$i.dae
end
  


  

