#!/bin/tcsh
# make-lagemann.sh: copy template Collada (cube.dae) and apply
# sed magic to replace the dummy textures with actual scanned
# textures.
echo "make-lagemann.sh"


# Philipp's Layout is: 
# 1=back@180 2=cover@0 3=top@90 4=bottom@270 5=spine@-90 6=front-edge@90
#

# lagemann-front-edge.jpg: 115 x 1886
# part 1: cover .. page 50 of 300
# part 2: page 51 .. page 80 of 300
# part 3: page 81 .. page 130 of 300
# part 4: page 131 .. 300 of 300
jpegtopnm lagemann-front-edge.jpg | pnmcut -l   0 -r  19 | pnmflip -r90 | pnmtojpeg > lagemann-1-front.jpg
jpegtopnm lagemann-front-edge.jpg | pnmcut -l  20 -r  31 | pnmflip -r90 | pnmtojpeg > lagemann-2-front.jpg
jpegtopnm lagemann-front-edge.jpg | pnmcut -l  32 -r  50 | pnmflip -r90 | pnmtojpeg > lagemann-3-front.jpg
jpegtopnm lagemann-front-edge.jpg | pnmcut -l  51 -r 114 | pnmflip -r90 | pnmtojpeg > lagemann-4-front.jpg

# lagemann-spine.jpg: 88 x 1874
jpegtopnm lagemann-spine.jpg | pnmcut -l   0 -r  14 | pnmflip -r90 | pnmtojpeg > lagemann-1-spine.jpg
jpegtopnm lagemann-spine.jpg | pnmcut -l  15 -r  24 | pnmflip -r90 | pnmtojpeg > lagemann-2-spine.jpg
jpegtopnm lagemann-spine.jpg | pnmcut -l  25 -r  38 | pnmflip -r90 | pnmtojpeg > lagemann-3-spine.jpg
jpegtopnm lagemann-spine.jpg | pnmcut -l  39 -r  87 | pnmflip -r90 | pnmtojpeg > lagemann-4-spine.jpg

# lagemann-top.jpg: 1288 x 105
jpegtopnm lagemann-top.jpg | pnmcut -t   0 -b  18 | pnmtojpeg > lagemann-1-top.jpg
jpegtopnm lagemann-top.jpg | pnmcut -t  19 -b  28 | pnmtojpeg > lagemann-2-top.jpg
jpegtopnm lagemann-top.jpg | pnmcut -t  29 -b  46 | pnmtojpeg > lagemann-3-top.jpg
jpegtopnm lagemann-top.jpg | pnmcut -t  47 -b 104 | pnmtojpeg > lagemann-4-top.jpg

# lagemann-bottom.jpg: 1294 x 99
jpegtopnm lagemann-bottom.jpg | pnmcut -t   0 -b  16 | pnmflip -r180 | pnmtojpeg > lagemann-1-bottom.jpg
jpegtopnm lagemann-bottom.jpg | pnmcut -t  17 -b  26 | pnmflip -r180 | pnmtojpeg > lagemann-2-bottom.jpg
jpegtopnm lagemann-bottom.jpg | pnmcut -t  27 -b  43 | pnmflip -r180 | pnmtojpeg > lagemann-3-bottom.jpg
jpegtopnm lagemann-bottom.jpg | pnmcut -t  44 -b  98 | pnmflip -r180 | pnmtojpeg > lagemann-4-bottom.jpg

# covers of part 1..4
cp lagemann-cover.jpg   lagemann-1-cover.jpg
cp lagemann-page51.jpg  lagemann-2-cover.jpg
cp lagemann-page81.jpg  lagemann-3-cover.jpg
cp lagemann-page131.jpg lagemann-4-cover.jpg

# backs of part 1..4
jpegtopnm lagemann-page50.jpg  | pnmflip -r180 | pnmtojpeg > lagemann-1-back.jpg
jpegtopnm lagemann-page80.jpg  | pnmflip -r180 | pnmtojpeg > lagemann-2-back.jpg
jpegtopnm lagemann-page130.jpg | pnmflip -r180 | pnmtojpeg > lagemann-3-back.jpg
jpegtopnm lagemann-back.jpg    | pnmflip -r180 | pnmtojpeg > lagemann-4-back.jpg


# part 1
cp cube.dae /tmp/
sed -i s/1.png/lagemann-1-back.jpg/ /tmp/cube.dae
sed -i s/2.png/lagemann-1-cover.jpg/ /tmp/cube.dae
sed -i s/3.png/lagemann-1-top.jpg/ /tmp/cube.dae
sed -i s/4.png/lagemann-1-bottom.jpg/ /tmp/cube.dae
sed -i s/5.png/lagemann-1-spine.jpg/ /tmp/cube.dae
sed -i s/6.png/lagemann-1-front.jpg/ /tmp/cube.dae
cp /tmp/cube.dae book-lagemann-part1.dae

# part 2
cp cube.dae /tmp/
sed -i s/1.png/lagemann-2-back.jpg/ /tmp/cube.dae
sed -i s/2.png/lagemann-2-cover.jpg/ /tmp/cube.dae
sed -i s/3.png/lagemann-2-top.jpg/ /tmp/cube.dae
sed -i s/4.png/lagemann-2-bottom.jpg/ /tmp/cube.dae
sed -i s/5.png/lagemann-2-spine.jpg/ /tmp/cube.dae
sed -i s/6.png/lagemann-2-front.jpg/ /tmp/cube.dae
cp /tmp/cube.dae book-lagemann-part2.dae

# part 3
cp cube.dae /tmp/
sed -i s/1.png/lagemann-3-back.jpg/ /tmp/cube.dae
sed -i s/2.png/lagemann-3-cover.jpg/ /tmp/cube.dae
sed -i s/3.png/lagemann-3-top.jpg/ /tmp/cube.dae
sed -i s/4.png/lagemann-3-bottom.jpg/ /tmp/cube.dae
sed -i s/5.png/lagemann-3-spine.jpg/ /tmp/cube.dae
sed -i s/6.png/lagemann-3-front.jpg/ /tmp/cube.dae
cp /tmp/cube.dae book-lagemann-part3.dae

# part 4
cp cube.dae /tmp/
sed -i s/1.png/lagemann-4-back.jpg/ /tmp/cube.dae
sed -i s/2.png/lagemann-4-cover.jpg/ /tmp/cube.dae
sed -i s/3.png/lagemann-4-top.jpg/ /tmp/cube.dae
sed -i s/4.png/lagemann-4-bottom.jpg/ /tmp/cube.dae
sed -i s/5.png/lagemann-4-spine.jpg/ /tmp/cube.dae
sed -i s/6.png/lagemann-4-front.jpg/ /tmp/cube.dae
cp /tmp/cube.dae book-lagemann-part4.dae

# lagemann-cover.jpg
# lagemann-page50.jpg
# lagemann-page51.jpg
# lagemann-page80.jpg
# lagemann-page81.jpg
# lagemann-page130.jpg
# lagemann-page131.jpg
# lagemann-back.jpg

