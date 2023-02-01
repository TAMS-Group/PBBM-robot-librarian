#!/bin/tcsh
# make-caterpillar.sh: build Collada meshes for all pages of
# "The Very Hungry Caterpillar" by Eric Carle. 
# Scan found and downloaded from epdf.pub.
# 
# A big book is easier to manipulate with a robot :-) from:
# https://www.amazon.com/Very-Hungry-Caterpillar-Hardcover-Book/dp/B0035YD21W
# Brand Name 	Constructive Playthings
# Item Weight 	1.13 pounds                           0.51 kg
# Product Dimensions 	12.2 x 8.5 x 0.5 inches       0.310 x 0.216 x 0.0127 m
#
# Note that some pages are smaller than others! We also make cover
# and back-cover pages a little wider than the others.
# 
# Copy template Collada (cube.dae) and apply
# sed magic to replace the dummy textures with actual scanned
# textures.
#
# fnh, 30.11.2019
#
echo "make-caterpillar.sh"

# Philipp's cube layout is: 
# 1=back@180 2=cover@0 3=top@90 4=bottom@270 5=spine@-90 6=front-edge@90
#
# image sizes:
# cover/1  957x682
# 2        955x675
# 3        953x680
# 4        955x675
# 5        953x680
# 6        942x675
# 7        957x675
# 8        938x675
# 9        948x663 cropped
# 10       938x675 cropped
# 11       940x680 cropped
# 12       938x675
# 13       940x680
# 14       938x675
# 15       940x680
# 16       938x675
# 17       940x680
# 18       938x675
# 19       940x680
# 20       938x675
# 21       940x680
# 22       917x678
# 23       940x680
# 24       938x675
# 25       932x671
# 26/back  942x678
# 
# total 13 pages -> 13 parts
# parts: 1/2 3/4 5/6 ... 25/26#

# front-edge taken from "Lagemann" book, size 115x1886. Crop into 13 parts...
jpegtopnm caterpillar-front-edge.jpg | pnmcut -l   0 -r   9 | pnmflip -r90 | pnmtojpeg > caterpillar-01-front.jpg
jpegtopnm caterpillar-front-edge.jpg | pnmcut -l   9 -r  18 | pnmflip -r90 | pnmtojpeg > caterpillar-02-front.jpg
jpegtopnm caterpillar-front-edge.jpg | pnmcut -l  18 -r  27 | pnmflip -r90 | pnmtojpeg > caterpillar-03-front.jpg
jpegtopnm caterpillar-front-edge.jpg | pnmcut -l  27 -r  35 | pnmflip -r90 | pnmtojpeg > caterpillar-04-front.jpg
jpegtopnm caterpillar-front-edge.jpg | pnmcut -l  35 -r  44 | pnmflip -r90 | pnmtojpeg > caterpillar-05-front.jpg
jpegtopnm caterpillar-front-edge.jpg | pnmcut -l  44 -r  53 | pnmflip -r90 | pnmtojpeg > caterpillar-06-front.jpg
jpegtopnm caterpillar-front-edge.jpg | pnmcut -l  53 -r  62 | pnmflip -r90 | pnmtojpeg > caterpillar-07-front.jpg
jpegtopnm caterpillar-front-edge.jpg | pnmcut -l  62 -r  70 | pnmflip -r90 | pnmtojpeg > caterpillar-08-front.jpg
jpegtopnm caterpillar-front-edge.jpg | pnmcut -l  70 -r  79 | pnmflip -r90 | pnmtojpeg > caterpillar-09-front.jpg
jpegtopnm caterpillar-front-edge.jpg | pnmcut -l  79 -r  88 | pnmflip -r90 | pnmtojpeg > caterpillar-10-front.jpg
jpegtopnm caterpillar-front-edge.jpg | pnmcut -l  88 -r  97 | pnmflip -r90 | pnmtojpeg > caterpillar-11-front.jpg
jpegtopnm caterpillar-front-edge.jpg | pnmcut -l  97 -r 106 | pnmflip -r90 | pnmtojpeg > caterpillar-12-front.jpg
jpegtopnm caterpillar-front-edge.jpg | pnmcut -l 106 -r 114 | pnmflip -r90 | pnmtojpeg > caterpillar-13-front.jpg

# caterpillar-spine-crop.jpg: 1112 x 80 
jpegtopnm caterpillar-spine-crop.jpg | pnmcut -l   0 -r   6 | pnmflip -r90 | pnmtojpeg > caterpillar-01-spine.jpg
jpegtopnm caterpillar-spine-crop.jpg | pnmcut -l   7 -r  12 | pnmflip -r90 | pnmtojpeg > caterpillar-02-spine.jpg
jpegtopnm caterpillar-spine-crop.jpg | pnmcut -l  13 -r  18 | pnmflip -r90 | pnmtojpeg > caterpillar-03-spine.jpg
jpegtopnm caterpillar-spine-crop.jpg | pnmcut -l  19 -r  24 | pnmflip -r90 | pnmtojpeg > caterpillar-04-spine.jpg
jpegtopnm caterpillar-spine-crop.jpg | pnmcut -l  25 -r  30 | pnmflip -r90 | pnmtojpeg > caterpillar-05-spine.jpg
jpegtopnm caterpillar-spine-crop.jpg | pnmcut -l  31 -r  37 | pnmflip -r90 | pnmtojpeg > caterpillar-06-spine.jpg
jpegtopnm caterpillar-spine-crop.jpg | pnmcut -l  38 -r  43 | pnmflip -r90 | pnmtojpeg > caterpillar-07-spine.jpg
jpegtopnm caterpillar-spine-crop.jpg | pnmcut -l  44 -r  49 | pnmflip -r90 | pnmtojpeg > caterpillar-08-spine.jpg
jpegtopnm caterpillar-spine-crop.jpg | pnmcut -l  50 -r  55 | pnmflip -r90 | pnmtojpeg > caterpillar-09-spine.jpg
jpegtopnm caterpillar-spine-crop.jpg | pnmcut -l  56 -r  62 | pnmflip -r90 | pnmtojpeg > caterpillar-10-spine.jpg
jpegtopnm caterpillar-spine-crop.jpg | pnmcut -l  63 -r  68 | pnmflip -r90 | pnmtojpeg > caterpillar-11-spine.jpg
jpegtopnm caterpillar-spine-crop.jpg | pnmcut -l  69 -r  73 | pnmflip -r90 | pnmtojpeg > caterpillar-12-spine.jpg
jpegtopnm caterpillar-spine-crop.jpg | pnmcut -l  74 -r  79 | pnmflip -r90 | pnmtojpeg > caterpillar-13-spine.jpg

# (lagemann)-topjpg: 1288 x 105
jpegtopnm caterpillar-top.jpg | pnmcut -t   0 -b   8 | pnmtojpeg > caterpillar-01-top.jpg
jpegtopnm caterpillar-top.jpg | pnmcut -t   9 -b  16 | pnmtojpeg > caterpillar-02-top.jpg
jpegtopnm caterpillar-top.jpg | pnmcut -t  17 -b  24 | pnmtojpeg > caterpillar-03-top.jpg
jpegtopnm caterpillar-top.jpg | pnmcut -t  25 -b  33 | pnmtojpeg > caterpillar-04-top.jpg
jpegtopnm caterpillar-top.jpg | pnmcut -t  34 -b  40 | pnmtojpeg > caterpillar-05-top.jpg
jpegtopnm caterpillar-top.jpg | pnmcut -t  41 -b  48 | pnmtojpeg > caterpillar-06-top.jpg
jpegtopnm caterpillar-top.jpg | pnmcut -t  49 -b  56 | pnmtojpeg > caterpillar-07-top.jpg
jpegtopnm caterpillar-top.jpg | pnmcut -t  57 -b  64 | pnmtojpeg > caterpillar-08-top.jpg
jpegtopnm caterpillar-top.jpg | pnmcut -t  65 -b  72 | pnmtojpeg > caterpillar-09-top.jpg
jpegtopnm caterpillar-top.jpg | pnmcut -t  73 -b  80 | pnmtojpeg > caterpillar-10-top.jpg
jpegtopnm caterpillar-top.jpg | pnmcut -t  81 -b  88 | pnmtojpeg > caterpillar-11-top.jpg
jpegtopnm caterpillar-top.jpg | pnmcut -t  89 -b  96 | pnmtojpeg > caterpillar-12-top.jpg
jpegtopnm caterpillar-top.jpg | pnmcut -t  97 -b 104 | pnmtojpeg > caterpillar-13-top.jpg

# (lagemann)-bottom.jpg: 1294 x 99
jpegtopnm caterpillar-bottom.jpg | pnmcut -t   0 -b  7 | pnmflip -r180 | pnmtojpeg > caterpillar-01-bottom.jpg
jpegtopnm caterpillar-bottom.jpg | pnmcut -t   8 -b 15 | pnmflip -r180 | pnmtojpeg > caterpillar-02-bottom.jpg
jpegtopnm caterpillar-bottom.jpg | pnmcut -t  16 -b 22 | pnmflip -r180 | pnmtojpeg > caterpillar-03-bottom.jpg
jpegtopnm caterpillar-bottom.jpg | pnmcut -t  23 -b 30 | pnmflip -r180 | pnmtojpeg > caterpillar-04-bottom.jpg
jpegtopnm caterpillar-bottom.jpg | pnmcut -t  31 -b 38 | pnmflip -r180 | pnmtojpeg > caterpillar-05-bottom.jpg
jpegtopnm caterpillar-bottom.jpg | pnmcut -t  39 -b 45 | pnmflip -r180 | pnmtojpeg > caterpillar-06-bottom.jpg
jpegtopnm caterpillar-bottom.jpg | pnmcut -t  46 -b 53 | pnmflip -r180 | pnmtojpeg > caterpillar-07-bottom.jpg
jpegtopnm caterpillar-bottom.jpg | pnmcut -t  54 -b 60 | pnmflip -r180 | pnmtojpeg > caterpillar-08-bottom.jpg
jpegtopnm caterpillar-bottom.jpg | pnmcut -t  61 -b 68 | pnmflip -r180 | pnmtojpeg > caterpillar-09-bottom.jpg
jpegtopnm caterpillar-bottom.jpg | pnmcut -t  69 -b 76 | pnmflip -r180 | pnmtojpeg > caterpillar-10-bottom.jpg
jpegtopnm caterpillar-bottom.jpg | pnmcut -t  77 -b 83 | pnmflip -r180 | pnmtojpeg > caterpillar-11-bottom.jpg
jpegtopnm caterpillar-bottom.jpg | pnmcut -t  84 -b 91 | pnmflip -r180 | pnmtojpeg > caterpillar-12-bottom.jpg
jpegtopnm caterpillar-bottom.jpg | pnmcut -t  92 -b 98 | pnmflip -r180 | pnmtojpeg > caterpillar-13-bottom.jpg


# "covers" of part 01 .. 13
cp  caterpillar-01.jpg  caterpillar-01-cover.jpg
cp  caterpillar-03.jpg  caterpillar-02-cover.jpg
cp  caterpillar-05.jpg  caterpillar-03-cover.jpg
cp  caterpillar-07.jpg  caterpillar-04-cover.jpg
cp  caterpillar-09-crop.jpg  caterpillar-05-cover.jpg
cp  caterpillar-11-crop.jpg  caterpillar-06-cover.jpg
cp  caterpillar-13-crop.jpg  caterpillar-07-cover.jpg
cp  caterpillar-15-crop.jpg  caterpillar-08-cover.jpg
cp  caterpillar-17.jpg  caterpillar-09-cover.jpg
cp  caterpillar-19.jpg  caterpillar-10-cover.jpg
cp  caterpillar-21.jpg  caterpillar-11-cover.jpg
cp  caterpillar-23.jpg  caterpillar-12-cover.jpg
cp  caterpillar-25.jpg  caterpillar-13-cover.jpg



# "backs" of part 01 .. 13
jpegtopnm caterpillar-02.jpg | pnmflip -r180 | pnmtojpeg > caterpillar-01-back.jpg
jpegtopnm caterpillar-04.jpg | pnmflip -r180 | pnmtojpeg > caterpillar-02-back.jpg
jpegtopnm caterpillar-06.jpg | pnmflip -r180 | pnmtojpeg > caterpillar-03-back.jpg
jpegtopnm caterpillar-08.jpg | pnmflip -r180 | pnmtojpeg > caterpillar-04-back.jpg
jpegtopnm caterpillar-10-crop.jpg | pnmflip -r180 | pnmtojpeg > caterpillar-05-back.jpg
jpegtopnm caterpillar-12-crop.jpg | pnmflip -r180 | pnmtojpeg > caterpillar-06-back.jpg
jpegtopnm caterpillar-14-crop.jpg | pnmflip -r180 | pnmtojpeg > caterpillar-07-back.jpg
jpegtopnm caterpillar-16-crop.jpg | pnmflip -r180 | pnmtojpeg > caterpillar-08-back.jpg
jpegtopnm caterpillar-18.jpg | pnmflip -r180 | pnmtojpeg > caterpillar-09-back.jpg
jpegtopnm caterpillar-20.jpg | pnmflip -r180 | pnmtojpeg > caterpillar-10-back.jpg
jpegtopnm caterpillar-22.jpg | pnmflip -r180 | pnmtojpeg > caterpillar-11-back.jpg
jpegtopnm caterpillar-24.jpg | pnmflip -r180 | pnmtojpeg > caterpillar-12-back.jpg
jpegtopnm caterpillar-26.jpg | pnmflip -r180 | pnmtojpeg > caterpillar-13-back.jpg


foreach i (01 02 03 04 05 06 07 08 09 10 11 12 13)
echo $i
cp cube.dae /tmp/
sed -i s/1.png/caterpillar-$i-back.jpg/ /tmp/cube.dae
sed -i s/2.png/caterpillar-$i-cover.jpg/ /tmp/cube.dae
sed -i s/3.png/caterpillar-$i-top.jpg/ /tmp/cube.dae
sed -i s/4.png/caterpillar-$i-bottom.jpg/ /tmp/cube.dae
sed -i s/5.png/caterpillar-$i-spine.jpg/ /tmp/cube.dae
sed -i s/6.png/caterpillar-$i-front.jpg/ /tmp/cube.dae
cp /tmp/cube.dae book-caterpillar-part-$i.dae
end



# part 2
#cp cube.dae /tmp/
#sed -i s/1.png/lagemann-2-back.jpg/ /tmp/cube.dae
#sed -i s/2.png/lagemann-2-cover.jpg/ /tmp/cube.dae
#sed -i s/3.png/lagemann-2-top.jpg/ /tmp/cube.dae
#sed -i s/4.png/lagemann-2-bottom.jpg/ /tmp/cube.dae
#sed -i s/5.png/lagemann-2-spine.jpg/ /tmp/cube.dae
#sed -i s/6.png/lagemann-2-front.jpg/ /tmp/cube.dae
#cp /tmp/cube.dae book-lagemann-part2.dae

echo "ok." 
