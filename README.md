# Problem 1
# Problem 2
## original map
1    1    1    1    1    1    1    1      
1    0    0    1    0    0    0    1      
1    0    0    1    0    0    0    1      
1    0    0    1    0    0    0    1      
1    0    0    1    0    0    0    1      
1    0    1    0    0    0    0    1      
1    0    1    0    0    0    0    1      
1    0    1    1    1    1    0    1      
1    0    0    0    0    0    0    1      
1    1    1    1    1    1    1    1      
total free spaces: 38
number of rows:10, number of columns:8
## step number 1
current convex cover index is 2
start at, i:8, j:6  
1    1    1    1    1    1    1    1      
1    0    0    1    0    0    2    1      
1    0    0    1    0    0    2    1      
1    0    0    1    0    0    2    1      
1    0    0    1    0    0    2    1      
1    0    1    0    0    0    2    1      
1    0    1    0    0    0    2    1      
1    0    1    1    1    1    2    1      
1    0    0    0    0    0    2    1      
1    1    1    1    1    1    1    1      
current convex cover boundaries, up:1, down:8, left:6, right:6
## step number 2
current convex cover index is 3
start at, i:2, j:5  
1    1    1    1    1    1    1    1      
1    0    0    1    3    3    3    1      
1    0    0    1    3    3    3    1      
1    0    0    1    3    3    3    1      
1    0    0    1    3    3    3    1      
1    0    1    0    3    3    3    1      
1    0    1    0    3    3    3    1      
1    0    1    1    1    1    0    1      
1    0    0    0    0    0    0    1      
1    1    1    1    1    1    1    1      
current convex cover boundaries, up:1, down:6, left:4, right:6
## step number 3
current convex cover index is 4
start at, i:5, j:1  
1    1    1    1    1    1    1    1      
1    4    0    1    0    0    0    1      
1    4    0    1    0    0    0    1      
1    4    0    1    0    0    0    1      
1    4    0    1    0    0    0    1      
1    4    1    0    0    0    0    1      
1    4    1    0    0    0    0    1      
1    4    1    1    1    1    0    1      
1    4    0    0    0    0    0    1      
1    1    1    1    1    1    1    1      
current convex cover boundaries, up:1, down:8, left:1, right:1
## step number 4
current convex cover index is 5
start at, i:8, j:3  
1    1    1    1    1    1    1    1      
1    0    0    1    0    0    0    1      
1    0    0    1    0    0    0    1      
1    0    0    1    0    0    0    1      
1    0    0    1    0    0    0    1      
1    0    1    0    0    0    0    1      
1    0    1    0    0    0    0    1      
1    0    1    1    1    1    0    1      
1    5    5    5    5    5    5    1      
1    1    1    1    1    1    1    1      
current convex cover boundaries, up:8, down:8, left:1, right:6
## step number 5
current convex cover index is 6
start at, i:4, j:2  
1    1    1    1    1    1    1    1      
1    6    6    1    0    0    0    1      
1    6    6    1    0    0    0    1      
1    6    6    1    0    0    0    1      
1    6    6    1    0    0    0    1      
1    0    1    0    0    0    0    1      
1    0    1    0    0    0    0    1      
1    0    1    1    1    1    0    1      
1    0    0    0    0    0    0    1      
1    1    1    1    1    1    1    1      
current convex cover boundaries, up:1, down:4, left:1, right:2
## step number 6
current convex cover index is 7
start at, i:5, j:3  
1    1    1    1    1    1    1    1      
1    0    0    1    0    0    0    1      
1    0    0    1    0    0    0    1      
1    0    0    1    0    0    0    1      
1    0    0    1    0    0    0    1      
1    0    1    7    7    7    7    1      
1    0    1    7    7    7    7    1      
1    0    1    1    1    1    0    1      
1    0    0    0    0    0    0    1      
1    1    1    1    1    1    1    1      
current convex cover boundaries, up:5, down:6, left:3, right:6
convex cover boundary has size:6
## Done
Now that every free space of the original map is covered by at least one convex cover, the process stops. In Unity, the List<Tuple<int,int,int,int>> variable convex_cover_boundary contains the something like "up:1, down:8, left:6, right:6". Each colors square shown in Unity covers the "0" spaces in that particular convex cover. 
