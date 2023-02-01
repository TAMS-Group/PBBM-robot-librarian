# One polygon is made up of 4 vertices containing xy-coordinates:
# ((x1, y1), (x2, y2), (x3, y3), (x4, y4)) from left top as 
# first point and do clock-wise count for two, three, and four.
# 
# Because message files can only contain 1D-arrays, each polygon
# is spread out across the following 4 arrays. 
# 
# Naturally, the polygons are determined by the index meaning 
# To assemble the first polygon, pick the coordinated from all
# arrays at index 0.

int32[] polys_x1 # x coordinate of the 1st vertex of all polys
int32[] polys_x2 # x coordinate of the 2nd vertex of all polys
int32[] polys_x3 # x coordinate of the 3rd vertex of all polys
int32[] polys_x4 # x coordinate of the 4th vertex of all polys
int32[] polys_y1 # y coordinate of the 1st vertex of all polys
int32[] polys_y2 # y coordinate of the 2nd vertex of all polys
int32[] polys_y3 # y coordinate of the 3rd vertex of all polys
int32[] polys_y4 # y coordinate of the 4th vertex of all polys
