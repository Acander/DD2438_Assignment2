# Problem 1
# Problem 2
## original map
number of rows:10, number of columns:8
original map
1, 1, 1, 1, 1, 1, 1, 1,
1, 0, 0, 1, 0, 0, 0, 1,
1, 0, 0, 1, 0, 0, 0, 1,
1, 0, 0, 1, 0, 0, 0, 1,
1, 0, 0, 1, 0, 0, 0, 1,
1, 0, 1, 0, 0, 0, 0, 1,
1, 0, 1, 0, 0, 0, 0, 1,
1, 0, 1, 1, 1, 1, 0, 1,
1, 0, 0, 0, 0, 0, 0, 1,
1, 1, 1, 1, 1, 1, 1, 1,
total free spaces: 38
## first step

current convex cover index is 2
start at, i:6, j:1
1, 1, 1, 1, 1, 1, 1, 1,   
1, 2, 0, 1, 0, 0, 0, 1,   
1, 2, 0, 1, 0, 0, 0, 1,   
1, 2, 0, 1, 0, 0, 0, 1,   
1, 2, 0, 1, 0, 0, 0, 1,   
1, 2, 1, 0, 0, 0, 0, 1,   
1, 2, 1, 0, 0, 0, 0, 1,   
1, 2, 1, 1, 1, 1, 0, 1,   
1, 2, 0, 0, 0, 0, 0, 1,   
1, 1, 1, 1, 1, 1, 1, 1,   
current convex cover boundaries, up:1, down:8, left:1, right:1
## second step

current convex cover index is 3
start at, i:4, j:4
1, 1, 1, 1, 1, 1, 1, 1,   
1, 0, 0, 1, 3, 3, 3, 1,   
1, 0, 0, 1, 3, 3, 3, 1,   
1, 0, 0, 1, 3, 3, 3, 1,   
1, 0, 0, 1, 3, 3, 3, 1,   
1, 0, 1, 0, 3, 3, 3, 1,   
1, 0, 1, 0, 3, 3, 3, 1,   
1, 0, 1, 1, 1, 1, 0, 1,   
1, 0, 0, 0, 0, 0, 0, 1,   
1, 1, 1, 1, 1, 1, 1, 1,   
current convex cover boundaries, up:1, down:6, left:4, right:6
## third step

current convex cover index is 4
start at, i:3, j:2
1, 1, 1, 1, 1, 1, 1, 1,   
1, 4, 4, 1, 0, 0, 0, 1,   
1, 4, 4, 1, 0, 0, 0, 1,   
1, 4, 4, 1, 0, 0, 0, 1,   
1, 4, 4, 1, 0, 0, 0, 1,   
1, 0, 1, 0, 0, 0, 0, 1,   
1, 0, 1, 0, 0, 0, 0, 1,   
1, 0, 1, 1, 1, 1, 0, 1,   
1, 0, 0, 0, 0, 0, 0, 1,   
1, 1, 1, 1, 1, 1, 1, 1,   
current convex cover boundaries, up:1, down:4, left:1, right:2
## fourth step

current convex cover index is 5
start at, i:8, j:4
1, 1, 1, 1, 1, 1, 1, 1,   
1, 0, 0, 1, 0, 0, 0, 1,   
1, 0, 0, 1, 0, 0, 0, 1,   
1, 0, 0, 1, 0, 0, 0, 1,   
1, 0, 0, 1, 0, 0, 0, 1,   
1, 0, 1, 0, 0, 0, 0, 1,   
1, 0, 1, 0, 0, 0, 0, 1,   
1, 0, 1, 1, 1, 1, 0, 1,   
1, 5, 5, 5, 5, 5, 5, 1,   
1, 1, 1, 1, 1, 1, 1, 1,   
current convex cover boundaries, up:8, down:8, left:1, right:6
## fifth step

current convex cover index is 6
start at, i:5, j:3
1, 1, 1, 1, 1, 1, 1, 1,   
1, 0, 0, 1, 0, 0, 0, 1,   
1, 0, 0, 1, 0, 0, 0, 1,   
1, 0, 0, 1, 0, 0, 0, 1,   
1, 0, 0, 1, 0, 0, 0, 1,   
1, 0, 1, 6, 6, 6, 6, 1,   
1, 0, 1, 6, 6, 6, 6, 1,   
1, 0, 1, 1, 1, 1, 0, 1,   
1, 0, 0, 0, 0, 0, 0, 1,   
1, 1, 1, 1, 1, 1, 1, 1,   
current convex cover boundaries, up:5, down:6, left:3, right:6
## sixth step

current convex cover index is 7
start at, i:7, j:6
1, 1, 1, 1, 1, 1, 1, 1,   
1, 0, 0, 1, 0, 0, 7, 1,   
1, 0, 0, 1, 0, 0, 7, 1,   
1, 0, 0, 1, 0, 0, 7, 1,   
1, 0, 0, 1, 0, 0, 7, 1,   
1, 0, 1, 0, 0, 0, 7, 1,   
1, 0, 1, 0, 0, 0, 7, 1,   
1, 0, 1, 1, 1, 1, 7, 1,   
1, 0, 0, 0, 0, 0, 7, 1,   
1, 1, 1, 1, 1, 1, 1, 1,   
current convex cover boundaries, up:1, down:8, left:6, right:6
convex cover boundary has size:6
