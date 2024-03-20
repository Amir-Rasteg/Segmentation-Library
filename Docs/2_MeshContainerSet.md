Holds multiple MeshContainers of the same area. Useful if you have multiple scans of the exact same object/area and wish to do some statistical analysis on them

## Init / creation:

Requires list of mesh containers

Can optionally take in debug boolean toggle and linear calibration factor that will be applied to all MeshContainers. Without this all MeshContainers will have these values unchanged, even if they differ from each other (which in the case of the linear calibration factor will likely cause accuracy issues).

Can take a float of the true surface area / known surface area of the object, if supplied ratio measurements are possible.

Optionally can take the name of the mesh for better plot title generation, but this doesn't impact any data.

## Mass Events:

#### MassRemoveBlackRegions()
Takes: Optional maxBrightness : float = 0.01

returns a MeshContainerSet of the same MeshContainers but with all dark vertices removed

#### MassManualDefineBoxCrop()
Takes: Optional cubeHalfOffsetHeight : float = 0.1, Optional repressPrint : bool = False

For defining the box crop points of every MeshContainer in the set

#### MassManual2PPSSegmentation()
Takes: Optional repressPrint : bool = False

Multi-Step 2PPS segmentation for all MeshContainers in a set. Each meshContainer can have its own unique number of cropping steps

## Updating MeshContainers:

#### UpdateLinearCalibrations(CalibrationValue : float)
Takes: CalibrationValue : float

updates all contained MeshContainers linear calibration value.

#### UpdateDebugs( bool )
Takes: bool

updates all contained MeshContainers debug value.

## Plotting:

#### PlotSurfaceAreas()
Takes: Nothing!

Outputs a boxplot of the surface areas of the meshContainers, scaled with the linear calibration factor.

#### PlotSurfaceAreaRatios()
Takes: Nothing!

Outputs a boxplot of the surface areas of the meshContainers, scaled with the linear calibration factor, divided by the true surface area.\
Returns an error if no true surface area has been defined for the MeshContainerSet object.

## Properties (all can be retrieved, only few set):

#### MeshContainers
Takes: 

The list of MeshContainers originally inputted into this set, in the same order.

#### trueSurfaceArea
Takes: 

The true surface area of the mesh section in mm^2

#### size
Takes: 

Number of contained MeshContainers

#### surfaceAreas
Takes: 

1D numpy array of surface areas in order of the input list

#### surfaceAreasQuartiles
Takes: 

2D numpy array of surface areas in order of the input list. structured as \[ \[Q1,Q2,Q3\]....\[Q1,Q2,Q3\]\]

#### surfaceAreaRatios
Takes: 

1D numpy array of surface areas divided by the true surface area, in order of the input list. Errors without a true surface area defined.

#### surfaceAreasQuartiles
Takes: 

2D numpy array of surface areas divided by the true surface area, in order of the input list. structured as \[ \[Q1,Q2,Q3\]....\[Q1,Q2,Q3\]\]. Errors without a true surface area defined.