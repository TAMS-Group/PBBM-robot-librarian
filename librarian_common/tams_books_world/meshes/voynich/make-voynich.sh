#!/bin/tcsh
# make-voynich.sh: copy template Collada (cube.dae) and apply
# sed magic to replace the dummy textures with actual scanned
# textures.
echo "make-voynish.sh"


# Philipp's Layout is: 
# 1=back@180 2=cover@0 3=top@90 4=bottom@270 5=spine@-90 6=front-edge@90
#

# voynich-front-edge.jpg: 423 x 1734
# part 1: page1-40 of 100
# part 2: page 41..66 of 100
# part 3: page 67..80 of 100
# part 4: page 81..end of 100
jpegtopnm voynich-front-edge.jpg | pnmflip -r90 | pnmtojpeg > voynich-front-edge-90.jpg
jpegtopnm voynich-front-edge.jpg | pnmcut -l   0 -r 169 | pnmflip -r90 | pnmtojpeg > voynich-1-front.jpg
jpegtopnm voynich-front-edge.jpg | pnmcut -l 170 -r 279 | pnmflip -r90 | pnmtojpeg > voynich-2-front.jpg
jpegtopnm voynich-front-edge.jpg | pnmcut -l 270 -r 339 | pnmflip -r90 | pnmtojpeg > voynich-3-front.jpg
jpegtopnm voynich-front-edge.jpg | pnmcut -l 340 -r 422 | pnmflip -r90 | pnmtojpeg > voynich-4-front.jpg

# voynich-spine.jpg: 372x1815
jpegtopnm voynich-spine.jpg | pnmflip -r90 | pnmtojpeg > voynich-spine-90.jpg
jpegtopnm voynich-spine.jpg | pnmcut -l   0 -r 148 | pnmflip -r90 | pnmtojpeg > voynich-1-spine.jpg
jpegtopnm voynich-spine.jpg | pnmcut -l 149 -r 223 | pnmflip -r90 | pnmtojpeg > voynich-2-spine.jpg
jpegtopnm voynich-spine.jpg | pnmcut -l 224 -r 297 | pnmflip -r90 | pnmtojpeg > voynich-3-spine.jpg
jpegtopnm voynich-spine.jpg | pnmcut -l 298 -r 371 | pnmflip -r90 | pnmtojpeg > voynich-4-spine.jpg

# voynich-top.jpg: 1466x512
jpegtopnm voynich-top.jpg | pnmcut -t   0 -b 204 | pnmtojpeg > voynich-4-top.jpg
jpegtopnm voynich-top.jpg | pnmcut -t 205 -b 307 | pnmtojpeg > voynich-3-top.jpg
jpegtopnm voynich-top.jpg | pnmcut -t 308 -b 409 | pnmtojpeg > voynich-2-top.jpg
jpegtopnm voynich-top.jpg | pnmcut -t 410 -b 511 | pnmtojpeg > voynich-1-top.jpg

# voynich-bottom.jpg: 1441x454
jpegtopnm voynich-bottom.jpg | pnmflip -r180 | pnmtojpeg > voynich-bottom-180.jpg
jpegtopnm voynich-bottom.jpg | pnmcut -t   0 -b 181 | pnmflip -r180 | pnmtojpeg > voynich-1-bottom.jpg
jpegtopnm voynich-bottom.jpg | pnmcut -t 182 -b 272 | pnmflip -r180 | pnmtojpeg > voynich-2-bottom.jpg
jpegtopnm voynich-bottom.jpg | pnmcut -t 273 -b 363 | pnmflip -r180 | pnmtojpeg > voynich-3-bottom.jpg
jpegtopnm voynich-bottom.jpg | pnmcut -t 364 -b 453 | pnmflip -r180 | pnmtojpeg > voynich-4-bottom.jpg

# covers of part 1..4
cp voynich-cover.jpg  voynich-1-cover.jpg
cp voynich-page41.jpg voynich-2-cover.jpg
cp voynich-page67.jpg voynich-3-cover.jpg
cp voynich-page81.jpg voynich-4-cover.jpg

# backs of part 1..4
jpegtopnm voynich-page40.jpg | pnmflip -r180 | pnmtojpeg > voynich-1-back.jpg
jpegtopnm voynich-page66.jpg | pnmflip -r180 | pnmtojpeg > voynich-2-back.jpg
jpegtopnm voynich-page80.jpg | pnmflip -r180 | pnmtojpeg > voynich-3-back.jpg
jpegtopnm voynich-back.jpg   | pnmflip -r180 | pnmtojpeg > voynich-4-back.jpg


# part 1
cp cube.dae /tmp/
sed -i s/1.png/voynich-1-back.jpg/ /tmp/cube.dae
sed -i s/2.png/voynich-1-cover.jpg/ /tmp/cube.dae
sed -i s/3.png/voynich-1-top.jpg/ /tmp/cube.dae
sed -i s/4.png/voynich-1-bottom.jpg/ /tmp/cube.dae
sed -i s/5.png/voynich-1-spine.jpg/ /tmp/cube.dae
sed -i s/6.png/voynich-1-front.jpg/ /tmp/cube.dae
cp /tmp/cube.dae book-voynich-part1.dae

# part 2
cp cube.dae /tmp/
sed -i s/1.png/voynich-2-back.jpg/ /tmp/cube.dae
sed -i s/2.png/voynich-2-cover.jpg/ /tmp/cube.dae
sed -i s/3.png/voynich-2-top.jpg/ /tmp/cube.dae
sed -i s/4.png/voynich-2-bottom.jpg/ /tmp/cube.dae
sed -i s/5.png/voynich-2-spine.jpg/ /tmp/cube.dae
sed -i s/6.png/voynich-2-front.jpg/ /tmp/cube.dae
cp /tmp/cube.dae book-voynich-part2.dae

# part 3
cp cube.dae /tmp/
sed -i s/1.png/voynich-3-back.jpg/ /tmp/cube.dae
sed -i s/2.png/voynich-3-cover.jpg/ /tmp/cube.dae
sed -i s/3.png/voynich-3-top.jpg/ /tmp/cube.dae
sed -i s/4.png/voynich-3-bottom.jpg/ /tmp/cube.dae
sed -i s/5.png/voynich-3-spine.jpg/ /tmp/cube.dae
sed -i s/6.png/voynich-3-front.jpg/ /tmp/cube.dae
cp /tmp/cube.dae book-voynich-part3.dae

# part 4
cp cube.dae /tmp/
sed -i s/1.png/voynich-4-back.jpg/ /tmp/cube.dae
sed -i s/2.png/voynich-4-cover.jpg/ /tmp/cube.dae
sed -i s/3.png/voynich-4-top.jpg/ /tmp/cube.dae
sed -i s/4.png/voynich-4-bottom.jpg/ /tmp/cube.dae
sed -i s/5.png/voynich-4-spine.jpg/ /tmp/cube.dae
sed -i s/6.png/voynich-4-front.jpg/ /tmp/cube.dae
cp /tmp/cube.dae book-voynich-part4.dae


# complete book
cp cube.dae /tmp/
sed -i s/1.png/voynich-back.jpg/ /tmp/cube.dae
sed -i s/2.png/voynich-cover.jpg/ /tmp/cube.dae
sed -i s/3.png/voynich-top.jpg/ /tmp/cube.dae
sed -i s/4.png/voynich-bottom-180.jpg/ /tmp/cube.dae
sed -i s/5.png/voynich-spine-90.jpg/ /tmp/cube.dae
sed -i s/6.png/voynich-front-edge-90.jpg/ /tmp/cube.dae
cp /tmp/cube.dae book-voynich.dae

# voynich-cover.jpg
# voynich-page40.jpg
# voynich-page41.jpg
# voynich-page66.jpg
# voynich-page67.jpg
# voynich-page80.jpg
# voynich-page81.jpg
# voynich-back.jpg

