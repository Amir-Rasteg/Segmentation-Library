Holds the data and methods relevant to a single open3D triangle mesh.

The following is a list of public methods / properties available.

# Init / creation:

Requires open3D triangle mesh or string path to a ply file to import as a mesh. If importing from a path, will clean the mesh first.

Has optional options for enabling debug mode (printing more step by step), and linear calibration overrides.

### Init
* mesh (str path to ply file OR Open3D.geometry.TriangleMesh)
    * Needed for obvious reasons
* debug (bool)
	* Optional, default False
    * Prints additional statements while processing
* cleanOnImport (bool)
	* Optional, default True
    * When loading from a path, will clean mesh by removing duplicate vertices, degenerated triangles, and unreferenced vertices
* linearCalibration (float)
	* Optional (default 1.0)
    * This is the LINEAR calibration factor for scaling the model. If you calculated your calibration factor from areas, make sure to take the square root first!

Default constructor Method

* returns nothing

# Visualization:

### VisualizeMesh()
* No Arguments Taken

Pauses execution and shows a 3d window of the triangle mesh of the Mesh Container. Execution resumes once window is closed.

* returns nothing

### VisualizeNewCoordinatesWithMesh()
* Takes Coordinates (np.ndarray)
	* 2D array with XYZ and length equivalent to the number of points

Given a set of coordinates, renders the Mesh Containers triangleMesh + the coordinates as well.

* returns nothing

### VisualizeSelectedPoints()
* selectedPointIndexesSets (list\['str names of pickedPointSets'\])
	* list of string names of currently set PPSs

Similar to above, except instead of visualizing a single array of 3D coordinates, you can visualize (on top of the MeshContainers triangleMesh) a list of PickedPointSets by string name. Each set will be a different random color

* returns nothing

# PickedPointSets:

### GetPickedPoints()
* pickedPointSetName (str)
	* name of the pickedPointSet
* numPoints (int)
	* number of points in your PPS
* shouldForceNew (bool)
	* Optional, default false
    * if true, will overwrite any PPS of the same name, otherwise it will simply return with the one that exists

prompts the user to create a PickedPointSet of given name, containing number of points, overwriting the previous PickedPointSet of the same name if it already exists within the object itself (only if ShouldForceNew be True). The PickedPointSet is stored within the object in the Mesh Containers dictionary property by the name of "PickedPointSet**s**" using the pickedPointSetName as the key, and it is also returned by the method call itself.

* returns the generated (or retrieved) PPS. (PPS also stored in objects internal dictionary)

### SetPickedPoints()
* pickedPointSetName (str)
	* what to name the PPS
* pointIndexes (np.ndarray)
	* int 1D array of all vertex indexes to be included in the PPS

Instead of the user being prompted, a numpy array of point Indexes can be inputted directly into this function to create a pickedPointSet that will be stored in the Mesh Containers, using the key name provided. _Useful for automation of tasks with predefined points!_

* returns generated PPS (PPS also stored in objects internal dictionary)

# Measurement:

### FindDepth()
* planePointsName (str)
	* optional
    * Can be set to the name of an already existing PPS for automation purposes
* depthPointName (str)
	* optional
    * Can be set to the name of an already existing PPS for automation purposes

First prompts the user to select 3 points to act as a bottom plane, then asks to user to pick a single point. the height from this point to the plane is calculated, allowing for finding the depth (or height) of 3D features. This is done with the generation of temporary PickedPointSets, but by defining the optional variables, you can input your own PickedPointSets instead of prompting the user for automation purposes.

* returns depth (float) in mm

### GetDistanceBetweenPoints()
* numberOfPoints (int)
	* optional, default 2
    * used to determine the number of points to find the distance between (in sequential order)
* pickedPointSetPerimeterName (str)
	* optional
    * Can be set to the name of an already existing PPS for automation purposes
* close (bool)
	* optional, default False
    * also find the distance between the first and last point, useful for finding perimeters

Finds the distances between points. By default it is 2 points but you can increase it to more, and the distances from each point to the next will be summed and returned. Enabling "close" also adds the distance the last point to the first point, effectively creating a means to measure perimeter. As with above, a PickedPointSet can be defined for automation purposes.

* returns distancea (float) in mm

# Segmentation:
Note these functions output new MeshContainers, they do not modify the MeshContainer it is running on

### CropByBoundingBox()
* boundingBoxInitialPPSName (str)
	* optional
    * Can be set to the name of an already existing PPS for automation purposes
* cubeHalfOffsetHeight (float)
	* optional, defaults to 0.1
    * distance to extrude both ways. sometimes needs to be tuned if box cropping is not giving expected results
* previewCrop (bool)
	* optional, defaults to False
    * if True, will visualize the results after running the segmentation

User selects 4 points, and the region inside that section is cropped out and returned as another MeshContainer. The PickedPointSet can instead be predefined for automation purposes. Note that this current implementation is currently imperfect since it "rounds" the 4 points into a rectangle.

* returns a **single** MeshContainer that was cropped

### RemoveIslands()
* minimumArea (float)
	* optional, default 50.0
	* minimum area for a segment to be (in mm^2) to not be eliminated

Used to eliminate specks that often appear from the scanning and segmentation process

* returns a **single** MeshContainer that was cropped

### DivideByColorBandFilter()
* bottomBound (float from 0.0-1.0)
	* The bottom of the ColorBand
* topBound (float from 0.0-1.0)
	* The top of the ColorBand
* isbandPass (bool)
	* If true then the segmentation is BandPass, otherwise its BandReject / BandFilter
* colorChannel (str of a color or int of a colorIndex)
	* Color Channel to segment on


Color Channel BandPass/Reject filter. Top/Bottom bounds are for establishing the boundaries of the band, given the color values exist in a range between 0 and 1.0.

* returns list of \[PassedMeshContainer, RejectedMeshContainer\]

### RemoveCompletelyBlackPoints()
* maxBrightness (float)
	* optional, defaults to 0.01
    * anything darker than this value is eliminated

Due to scanning artifacts, some regions of scans may be pitch black. Use this to easily remove such regions if performing color segmentation

* returns list of \[PassedMeshContainer, RejectedMeshContainer\]

### DivideGiven2PPS()
* PPSInsideName (str)
	* optional
    * Can be set to the name of an already existing PPS for automation purposes
* PPSOutsideName (str)
	* optional
    * Can be set to the name of an already existing PPS for automation purposes
* numNewPoints (int)
	* optional, defaults to 4
    * number of points for user to pick (if user is picking)
* weights (np.ndarray)
	* optional, defaults to \[1,1,1,1\]
    	* first value weighs the importance of avoiding shared area
        * second value weighs the importance of increasing median distance
        * third and fourth values weights the importance on minimizing the inner and outer interquartile ranges respectfully
* numberAdjacentSearches (int)
	* optional, default 0
    * number of iterations to scan for adjacent point for each picked point, to quickly build up a data set for better analysis
    	* **WARNING** This algorithm isn't optimized at all and gets slower exponentially. It is best not to go over ~3


Given an inner and outer picked point set (if name not found, user will be prompted), find the color channel most likely to divide them by the Otsu method. Optional weights an be set to increase the importance of certain properties when comparing color channels. Once the best channel is found, performs an Otsu method threshold using the Picked points themselves (NOT the mesh as a whole). Capable of generating good crops on color-simple meshes


* returns list of \[PassedMeshContainer, RejectedMeshContainer\]

### DivideByOtsuColor()
* colorChannel (str of a color or int of a colorIndex)
	* colorChannel to perform segmentation on
* valToPass (float)
	* optional, defaults to 1.0
	* value that will be on the side of the Otsu threshold to pass

Divide the mesh via the Otsu method on a given color channel. The side that contains the valToPass will be the passed meshContainer, while the other will be the reject.

* returns list of \[PassedMeshContainer, RejectedMeshContainer\]

### DivideByOtsuColorSinglePickedPoint()
* colorChannel (str of a color or int of a colorIndex)
	* colorChannel to perform segmentation on
* singlePickedPointSetName (str)
	* optional
    * Can be set to the name of an already existing PPS for automation purposes
    
Just like the function above, but this time the valToPass comes from a user picked / defined pickedPoint.

* returns list of \[PassedMeshContainer, RejectedMeshContainer\]


# Wizard
### Wizard()
* takes nothing

Starts a terminal session in the console. Type in commands to perform various segmentations and visualizations. Type in 'Help' to see a list of available commands. Once you stop the wizard, the final mesh will be returned by this function, and a large dictionary will be printed into the console. You can copy and paste this dictionary into your script, and execute it later with the WizardLoad method to repeat the exact same steps

* returns cropped meshContainer
* prints out instructions to repeat experimnent with WizardLoad

### WizardLoad
* takes dictionary variable generated by Wizard

Repeats the exact steps in the dictionary

* returns cropped meshContainer

# etc Data Functions

### GetSpecificChannel
* colorChannel (str of a color or int of a colorIndex)
	* colorChannel to retrieve data on

for easily getting out color data

* returns 1D np.ndarray of selected channel across all vertices

### GetSpecificChannelQuartiles
* colorChannel (str of a color or int of a colorIndex)
	* colorChannel to retrieve Quartile data on

for easily getting out color quartile data

* returns 3 long 1D np.ndarray containing the first, second, and third quartile of a color

### Save
* folderPath (str)
	* folder path of where to save the file
* filename (str)
	* name of the file to save
    	* do not include '.ply'

used for saving the current meshContainer as a ply file to disk

* returns nothing
* writes to disk

### GenerateNewMeshWithColorRatio
* topColor (str of a color or int of a colorIndex)
	* The top color in the color ratio
* bottomColor (str of a color or int of a colorIndex)
	* The top color in the color ratio
* replaceNaNWith (float between 0.0 and 1.0)
	* optional, Default is 1.0
	* since when doing this ratio often causes NaNs due to division by 0 errors, NaNs are replaced with this value


Generates a new MeshContainer where the colors are generated by taking a defined 'top' color channel from the source container and dividing it by the 'bottom' color channel, all per vertex.


# Properties

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

### numVerts
int, number of vertices in mesh

### triangles
np.ndarray, triangle array from mesh (3 x number of triangles, each row composed of point indexes making up that triangle)

### triangles_RGB
np.ndarray, rgb value per triangle

### triangles_HSV
np.ndarray, hsv value per triangle

### surfaceArea
float, surface area of mesh scaled via linearCalibration value (squared), calculated by adding areas of all triangles

