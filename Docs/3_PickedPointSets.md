Objects stored within MeshContainers. Holds named sets of PickedPoints and properties to extract information about them. ONLY APPLIES TO THE MESH THE PICKED POINTS CAME FROM. Cropping or any function that changes vertex count on the parent mesh will make the current set invalid, which is why those operations result in new MeshContainers rather than modifying itself.


# Functions

### GetSpecificChannel()

Takes int or str.

returns a 1D numpy array of a specific color channel of all points in the picked points set

### GetSpecificChannelStat()

Takes int or str.

returns a 1D numpy array of quartiles 1, 2 and 3 of a specific color channel of all points in the picked points set

# Properties

## indexes

1D numpy array of the vertex indices that make up the set. Not Settable.

### coordinates

2D 3 x N numpy array of the vertex X Y Z positions in order of the indexes. Not Settable.

X, Y, Z  
... (number of points) long

### coordinates_Quartiles

2D 3 x 3 numpy array of the quartiles of all positions along the X Y and Z. Not Settable.

X_Q1, X_Q2, X_Q3  
Y_Q1, Y_Q2, Y_Q3  
Z_Q1, Z_Q2, Z_Q3  

### RGB

2D 3 x N numpy array of the vertex R G B colors in order of the indexes. Not Settable.

R, G, B  
... (number of points) long

### RGB_Quartiles

2D 3 x 3 numpy array of the quartiles of all RGB colors. Not Settable.

R_Q1, R_Q2, R_Q3  
G_Q1, G_Q2, G_Q3  
B_Q1, B_Q2, B_Q3  

### HSV

2D 3 x N numpy array of the vertex H S V colors in order of the indexes. Not Settable.

H, S, V  
... (number of points) long

### HSV_Quartiles

2D 3 x 3 numpy array of the quartiles of all HSV colors. Not Settable.

H_Q1, H_Q2, H_Q3  
S_Q1, S_Q2, S_Q3  
V_Q1, V_Q2, V_Q3  


### RGBHSV

2D 6 x n numpy array of the vertex R G B H S V colors in order of the indexes. Not Settable.

R, G, B, H, S, V  
... (number of points) long


### RGBHSV_Quartiles

2D 6 x 3 numpy array of the quartiles of all HSV colors. Not Settable.

R_Q1, R_Q2, R_Q3  
G_Q1, G_Q2, G_Q3  
B_Q1, B_Q2, B_Q3  
H_Q1, H_Q2, H_Q3  
S_Q1, S_Q2, S_Q3  
V_Q1, V_Q2, V_Q3  

### pointCount

int number of points in the set. Not Settable.