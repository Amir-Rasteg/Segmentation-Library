## Introduction

These set of scripts act as a wrapper around Open3D to add segmentation functions based on global color thresholding, and focused on medical usecases.

This system was built by Amir Rastegar as part of their Master's thesis at Duquesne University, under the supervision of Dr. Bin Yang

## Terminology

##### ColorIndexes:
In many areas of these scripts, color channels can either be defined as a string (red, green, blue, hue, saturation, value) or as an int (0-5 inclusive, matching red, green, blue, hue, saturation, value in order)

##### MeshContainer
Open3D Supports Triangle Meshes, as well as a few other mesh types. However, this is essentially raw data.

MeshContainers are the main class of this project, and are containers for the Open3D Triangle Mesh that add additional properties and methods relevant to color based cropping and segmentation.

MeshContainers are essentially immutable, and due to the use of caching, directly editing attributes or mesh properties directly may cause unexpected behavior. Instead, segmentation methods will output new MeshContainers as per the segmentation.

Keep in mind most segmentation methods output a list of 2 MeshContainers, with the first following the segmentation steps, and 2nd being the inverse of the defined segmentation. This may be useful for comparisons and other uses, but if you only need the desired semgneted area, you can retrieve it with

```
desiredArea, _ = fullMeshContainer.SegmentationMethod()
```

or if you do want both
```
desiredArea, undesiredArea = fullMeshContainer.SegmentationMethod()
```

##### PickedPointSet
Also abbreviated as PPS (or PP for a singular picked point), refers to a custom class called PickedPointSet that holds picked point information, as well as methods and properties to easily get information out of them. They generally should not be created directly, they are used by the MeshContainer internals directly