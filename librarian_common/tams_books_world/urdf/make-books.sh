#!/bin/tcsh
# make-books.sh: copy template Collada (cube.dae) and apply
# sed magic to replace the dummy textures with actual scanned
# textures.
echo "make-books.sh"

# Lagemann: Rechnerstrukturen, size 24.2 x 16.5 x 1.5 cm, 554 grams.
cp cube.urdf /tmp
sed -i s/book-link/book-lagemann-link/ /tmp/cube.urdf
sed -i s/book-mesh/lagemann.dae/ /tmp/cube.urdf
sed -i s/book-mass/0.554/ /tmp/cube.urdf
sed -i s/xscale/0.165/ /tmp/cube.urdf
sed -i s/yscale/0.242/ /tmp/cube.urdf
sed -i s/zscale/0.015/ /tmp/cube.urdf
sed -i s/xsize/0.165/ /tmp/cube.urdf
sed -i s/ysize/0.242/ /tmp/cube.urdf
sed -i s/zsize/0.015/ /tmp/cube.urdf
cp /tmp/cube.urdf book-lagemann.urdf

# Mölller: Rechnerstrukturen, size 23.6 x 15.3 + 2.3 cm, 592 grams.
cp cube.urdf /tmp
sed -i s/book-link/book-moeller-link/ /tmp/cube.urdf
sed -i s/book-mesh/moeller.dae/ /tmp/cube.urdf
sed -i s/book-mass/0.592/ /tmp/cube.urdf
sed -i s/xscale/0.153/ /tmp/cube.urdf
sed -i s/yscale/0.236/ /tmp/cube.urdf
sed -i s/zscale/0.023/ /tmp/cube.urdf
sed -i s/xsize/0.153/ /tmp/cube.urdf
sed -i s/ysize/0.236/ /tmp/cube.urdf
sed -i s/zsize/0.023/ /tmp/cube.urdf
cp /tmp/cube.urdf book-moeller.urdf



  

