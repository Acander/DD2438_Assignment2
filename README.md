# Problem 1
# Problem 2
An example of how convex cover is found is as followed: 
## original map
1 1 1 1 1 1 1 1 
1 0 0 1 0 0 0 1 
1 0 0 1 0 0 0 1 
1 0 0 1 0 0 0 1 
1 0 0 1 0 0 0 1 
1 0 1 0 0 0 0 1 
1 0 1 0 0 0 0 1 
1 0 1 1 1 1 0 1 
1 0 0 0 0 0 0 1 
1 1 1 1 1 1 1 1 
total free spaces: 38( "1" is obstacle, "0" is free space). The upper-left corner corresponds to (0,0) of (i,j). The lower-right corner is (9,7). There are 10 rows and 8 columns in this grid map. 
## First step 
current convex cover index is 2
start at, i:8, j:2
1 1 1 1 1 1 1 1 
1 0 0 1 0 0 0 1 
1 0 0 1 0 0 0 1 
1 0 0 1 0 0 0 1 
1 0 0 1 0 0 0 1 
1 0 1 0 0 0 0 1 
1 0 1 0 0 0 0 1 
1 0 1 1 1 1 0 1 
1 2 2 2 2 2 2 1 
1 1 1 1 1 1 1 1 
current convex cover boundaries, up:8, down:8, left:1, right:6
## Second step 
current convex cover index is 3
start at, i:5, j:5
1 1 1 1 1 1 1 1 
1 0 0 1 3 3 3 1 
1 0 0 1 3 3 3 1 
1 0 0 1 3 3 3 1 
1 0 0 1 3 3 3 1 
1 0 1 0 3 3 3 1 
1 0 1 0 3 3 3 1 
1 0 1 1 1 1 0 1 
1 0 0 0 0 0 0 1 
1 1 1 1 1 1 1 1 
current convex cover boundaries, up:1, down:6, left:4, right:6
## Third step 
current convex cover index is 4
start at, i:5, j:1
1 1 1 1 1 1 1 1 
1 4 0 1 0 0 0 1 
1 4 0 1 0 0 0 1 
1 4 0 1 0 0 0 1 
1 4 0 1 0 0 0 1 
1 4 1 0 0 0 0 1 
1 4 1 0 0 0 0 1 
1 4 1 1 1 1 0 1 
1 4 0 0 0 0 0 1 
1 1 1 1 1 1 1 1 
current convex cover boundaries, up:1, down:8, left:1, right:1
## Fourth step
current convex cover index is 5
start at, i:3, j:2
1 1 1 1 1 1 1 1 
1 5 5 1 0 0 0 1 
1 5 5 1 0 0 0 1 
1 5 5 1 0 0 0 1 
1 5 5 1 0 0 0 1 
1 0 1 0 0 0 0 1 
1 0 1 0 0 0 0 1 
1 0 1 1 1 1 0 1 
1 0 0 0 0 0 0 1 
1 1 1 1 1 1 1 1 
current convex cover boundaries, up:1, down:4, left:1, right:2
## Fifth step
current convex cover index is 6
start at, i:5, j:3
1 1 1 1 1 1 1 1 
1 0 0 1 0 0 0 1 
1 0 0 1 0 0 0 1 
1 0 0 1 0 0 0 1 
1 0 0 1 0 0 0 1 
1 0 1 6 6 6 6 1 
1 0 1 6 6 6 6 1 
1 0 1 1 1 1 0 1 
1 0 0 0 0 0 0 1 
1 1 1 1 1 1 1 1 
current convex cover boundaries, up:5, down:6, left:3, right:6
## Sixth step
current convex cover index is 7
start at, i:7, j:6
1 1 1 1 1 1 1 1 
1 0 0 1 0 0 7 1 
1 0 0 1 0 0 7 1 
1 0 0 1 0 0 7 1 
1 0 0 1 0 0 7 1 
1 0 1 0 0 0 7 1 
1 0 1 0 0 0 7 1 
1 0 1 1 1 1 7 1 
1 0 0 0 0 0 7 1 
1 1 1 1 1 1 1 1 
current convex cover boundaries, up:1, down:8, left:6, right:6
convex cover boundary has size:6
## Done
Now that every free space of the original map is covered by at least one convex cover, the process stops. In Unity, the List<Tuple<int,int,int,int>> variable convex_cover_boundary contains the something like "up:1, down:8, left:6, right:6". Each colors square shown in Unity covers the "0" spaces in that particular convex cover. 