#MeshContainerSet is a class that contains multiple MeshContainers of a single object. useful  for when you have multiple scans of the same object and wish to sample them

import numpy as np
import open3d as o3d
from MeshContainer import MeshContainer
import HelperFunctions as F;
import matplotlib.pyplot as plt
import copy;


class MeshContainerSet:
    
    
    def __init__(mcs, listofMeshContainers : list['MeshContainer'], linearCalibration : float = None, debug : bool = None, trueSurfaceArea : float = None, meshName : str = None):
        
        mcs.__MeshContainerList = listofMeshContainers;
        mcs.__trueSurfaceAreaHidden = trueSurfaceArea;
        mcs.__ResetCachedValues();
        mcs.meshName = meshName;
        
        if(linearCalibration is not None):
            mcs.UpdateLinearCalibrations(linearCalibration);
        #end
        
        if(debug is not None):
            mcs.UpdateDebugs(debug);
        #end
    #end
    
    def MassRemoveBlackRegions(mcs, maxBrightness : float = 0.01) -> 'MeshContainerSet':
        """
        Returns a MeshContainerSet based on this object but with the Black Regions Removed

        Parameters
        ----------
        maxBrightness : float, optional
            max brightness for a point before it is no longer cut off. The default is 0.01.
        Returns
        -------
        MeshContainerSet
            This objects MeshContainers but with black regions removed.

        """
        
        newMeshContainers : list = [];
        for MeCo in mcs.__MeshContainerList:
            cleanedMeshContainer, _ = MeCo.RemoveCompletelyBlackPoints(maxBrightness = maxBrightness);
            newMeshContainers.append(cleanedMeshContainer);
        #end
        
        return MeshContainerSet(newMeshContainers, trueSurfaceArea = mcs.__trueSurfaceAreaHidden, meshName = mcs.meshName);
    #end
    
    
    def MassManualDefineBoxCrop(mcs, cubeHalfOffsetHeight : float = 0.1, repressPrint : bool = False) -> 'MeshContainerSet':
        """
        For defining the box crop points of every MeshContainer in the set

        Parameters
        ----------
        cubeHalfOffsetHeight : float, optional
            Cropping Cube Height. Adjust if cropping is severely off. The default is 0.1.
        repressPrint : bool, optional
            prevent the printing of the produced array in console.. The default is False.

        Returns
        -------
        'MeshContainerSet'
            Set of Mesh Containers that have been cropped.

        """
        
        return mcs.__MassManualSingleLayerWork("BoxCrop", repressPrint, cubeHalfOffsetHeight = cubeHalfOffsetHeight);
    #end
    
    
    def __MassManualSingleLayerWork(mcs, workType : str, repressPrint : bool, cubeHalfOffsetHeight : float = 0.1, numPoints = 4) -> 'MeshContainerSet':
        """
        Given a task (single step), has users select required points, applies task, then asks user if its good before continuing, for every MeshContainer in the set.

        Parameters
        ----------
        workType : str
            What task is being done. Options are 'BoxCrop'.
        cubeHalfOffsetHeight : float, optional
            for BoxCrop Task. The default is 0.1.
        repressPrint : bool,
            prevent the printing of the produced array in console.
        numPoints : TYPE, optional
            number of points to pick per MeshContainer. The default is 4.

        Returns
        -------
        'MeshContainerSet'
            MeshContainerSet with the work applied.

        """
        i : int = 0;
        pickedPoints : list = [];
        MeshContainersOut : list = [];
        
        while i < mcs.size:
            meshContainer : MeshContainer = mcs.MeshContainers[i];
            pickedPointsTemp = (meshContainer.GetPickedPoints(workType, numPoints, ShouldForceNew = True).indexes); #Have User pick points
            
            if(workType == "BoxCrop"):
                meshContainer = meshContainer.CropByBoundingBox(boundingBoxInitialPPSName = workType, cubeHalfOffsetHeight = cubeHalfOffsetHeight); #apply crop
            #end
            
            meshContainer.VisualizeMesh();
            
            if(input("Type T if the action turned out well\n") == "T"):
                pickedPoints.append(pickedPointsTemp);
                MeshContainersOut.append(meshContainer);
                i = i + 1;
            #end
        #end
        
        name : str = "Unknown";
        if(mcs.meshName != "None"):
            name = mcs.meshName;
        #end
        
        if not repressPrint:  
            print("Points_" + workType + "_" + name + ": list = " + F.PointListToString(pickedPoints));
        #end
        
        return MeshContainerSet(MeshContainersOut, meshName = (name + "_" + workType));
    #end
    
    def MassManual2PPSSegmentation(mcs, repressPrint : bool = False, numberAdjacentSearches : int = 0) -> 'MeshContainerSet':
        """
        Multi-Step 2PPS segmentation for all MeshContainers in a set. Each meshContainer can have its own unique number of cropping steps

        Parameters
        ----------
        repressPrint : bool, optional
            prevent the printing of the produced array in console. The default is False.
        numberAdjacentSearches : int, optional
            number of adjacent points to include for eached pickjed point to build a better otsu model. Default is 0, please dont exceed 4.

        Returns
        -------
        'MeshContainerSet'
            The MeshContainerSet with all crops applied

        """
        return mcs.__MassManualMultiLayerWork("AutoSelect", repressPrint = repressPrint, numberAdjacentSearches = numberAdjacentSearches);
    #end
    
    def __MassManualMultiLayerWork(mcs, workType : str, repressPrint : bool, numberAdjacentSearches = 0) -> 'MeshContainerSet':
        """
        MultiStep Segmentation System. Each MeshContainer can have a unique number of segmentation steps

        Parameters
        ----------
        workType : str
            Type of segmentation. "AutoSelect" for 2PPS.
        repressPrint : bool
            prevent the printing of the produced array in console.
        numberAdjacentSearches : int, optional
            number of adjacent points to include for eached pickjed point to build a better otsu model. Default is 0, please dont exceed 4.

        Returns
        -------
        'MeshContainerSet'
            The MeshContainerSet with all crops applied

        """
        
        def ListListsArraysToString(layeredList : list):
            output : str = "[";
            for element in layeredList:
                output = output + F.PointListToString(element) + ",";
            #end
            output = output.rstrip(output[-1]);
            return (output + "]");
        #end
            
        
        PointsI_ml : list = [];
        PointsO_ml : list = [];
        finalMeshes : list = [];
        
        currentMeshIndex : int = 0;
        
        numPoints : int;
        if(workType == "AutoSelect"):
            numPoints = 4;
        #end
        
        while currentMeshIndex < mcs.size:
                        
            needsWork : bool = True;
            
            PointsI : list = [];
            PointsO : list = [];
            
            cachedMeshContainer : MeshContainer = copy.copy(mcs.MeshContainers[currentMeshIndex]); #current working mesh
            backUpMeshContainer : MeshContainer = copy.copy(mcs.MeshContainers[currentMeshIndex]); #for going one step back on redos
            
            while needsWork:
                
                ITemp = cachedMeshContainer.GetPickedPoints("PointsI", numPoints, ShouldForceNew = True).indexes;
                if(workType == "AutoSelect"):
                    OTemp = cachedMeshContainer.GetPickedPoints("PointsO", numPoints, ShouldForceNew = True).indexes;
                #end
                
                cachedMeshContainer, _ = cachedMeshContainer.DivideGiven2PPS("PointsI", "PointsO", numberAdjacentSearches = numberAdjacentSearches);
                
                cachedMeshContainer.VisualizeMesh();
                
                response = input("Type C to add another segmentation step, R to redo, or S to stop here with this MeshContainer\n");
                #breakpoint();
                if((response != "C") & (response != "S")):
                    response = "R"; #safety check
                #end
                
                if((response == "C") | (response == "S")):
                    PointsI.append(ITemp);
                    if(workType == "AutoSelect"):
                        PointsO.append(OTemp);
                    #end
                    backUpMeshContainer = copy.copy(cachedMeshContainer)
                #end
                
                if(response == "S"):
                    needsWork = False;
                #end
                
                if(response == "R"):
                    cachedMeshContainer = copy.copy(backUpMeshContainer);
                #end
            #end
            
            PointsI_ml.append(PointsI);
            if(workType == "AutoSelect"):
                PointsO_ml.append(PointsO);
            #end
            finalMeshes.append(cachedMeshContainer)
            currentMeshIndex = currentMeshIndex + 1;
        #end
        
        name : str = "Unknown";
        if(mcs.meshName != "None"):
            name = mcs.meshName;
        #end
        
        if not repressPrint:  
            print("MLPointsI_" + workType + "_" + name + ": list = " + ListListsArraysToString(PointsI_ml));
            if(workType == "AutoSelect"):
                print("MLPointsO_" + workType + "_" + name + ": list = " + ListListsArraysToString(PointsO_ml));
            #end
        #end
    
        return (MeshContainerSet(finalMeshes, meshName = (name + "_" + workType)))
    #end
    
    def MassApplyBoxCrop(mcs, listOfPPS : list[np.ndarray], subMeshName : str = "Unknown_BoxCropped") -> 'MeshContainerSet':
        """
        Given a list of PickedPointSets (likely generated from MassManualDefineBoxCrop), apply a box crop to all MeshContainers in this MeshContainerSet

        Parameters
        ----------
        listOfPPS : list
            list of picked point set indexes (each set in an np.ndarray).
        subMeshName : str, optional
            new name for the generated MeshContainers of the MeshContainerSet. The default is "Unknown_BoxCropped".

        Returns
        -------
        'MeshContainerSet'
            Generated MeshContainerSet.

        """
        meshContainersOut = [];

        
        for i, meshContainerToCrop in enumerate(mcs.MeshContainers):
            meshContainerToCrop.SetPickedPoints(subMeshName, listOfPPS[i] );
            meshContainersOut.append(meshContainerToCrop.CropByBoundingBox(boundingBoxInitialPPSName=subMeshName))
        #end
        
        return MeshContainerSet(meshContainersOut, meshName = subMeshName);
    #end
    
    def MassApply2PPSSegmentation(mcs, listOfPPSSetsIn : list[list], listOfPPSSetsOut : list[list], subMeshName : str = "Unknown_2PPSCropped", numberAdjacentSearches : int = 0) -> 'MeshContainerSet':
        """
        Given 2 lists of PickedPointSets (likely generated from MassManual2PPSSegmentation), apply a 2PPS segmentation to all MeshContainers in this MeshContainerSet. MultiStep Segementation

        Parameters
        ----------
        listOfPPSSetsIn : list[list]
            list of lists of picked point set indexes (each set in an np.ndarray).
        listOfPPSSetsOut : list[list]
            list of lists of picked point set indexes (each set in an np.ndarray).
        subMeshName : str, optional
            new name for the generated MeshContainers of the MeshContainerSet. The default is "Unknown_2PPSCropped".
        numberAdjacentSearches : int , optional
            number of adjacent points to include for eached pickjed point to build a better otsu model. Default is 0, please dont exceed 4.
        Returns
        -------
        'MeshContainerSet'
            Generated MeshContainerSet.

        """
        meshContainersOut : list = [];
        
        for meshContainerIndex, meshContainerToCrop in enumerate(mcs.MeshContainers):
            
            cacheMeshContainer = copy.copy(meshContainerToCrop);
            for PickedPointsIndexes in range(len(listOfPPSSetsIn[meshContainerIndex])):
                cacheMeshContainer.SetPickedPoints("inside", listOfPPSSetsIn[meshContainerIndex][PickedPointsIndexes]);
                cacheMeshContainer.SetPickedPoints("outside", listOfPPSSetsOut[meshContainerIndex][PickedPointsIndexes]);
                cacheMeshContainer, _ = cacheMeshContainer.DivideGiven2PPS(PPSInsideName = "inside", PPSOutsideName = "outside", numberAdjacentSearches = numberAdjacentSearches);
            #end
            meshContainersOut.append(cacheMeshContainer);
        #end
        return MeshContainerSet(meshContainersOut, meshName = subMeshName);
        
        
   
    def UpdateLinearCalibrations(mcs, newLinearCalibration : float):
        """
        Udpates the LinearCalibration value for all MeshContainers

        Parameters
        newLinearCalibration : float
            new linear calibration.

        """
        for mcs in mcs.__MeshContainerList:
            mcs.linearCalibration = newLinearCalibration;
        #end
    #end
    
    def UpdateDebugs(mcs, newDebugValue : bool):
        """
        Udpates the Debug value for all MeshContainers

        Parameters
        newDebugValue : bool
            new debug setting.

        """
        for mc in mcs.MeshContainerList:
            mc.debug = newDebugValue;
        #end
    
    #TODO add label for TrueSurfaceArea then update docs
    def PlotSurfaceAreas(mcs, TrueSurfaceArea : float = None):
        """
        Outputs a Boxplot of the Mesh Surface Areas
        """
        fig = plt.figure(figsize =(10, 7))
        ax = fig.add_subplot(111)
        ax.boxplot(mcs.surfaceAreas)
        
        if(mcs.meshName is None):
            ax.set_title('Surface Areas')
        else:
            ax.set_title('Surface Areas of ' + mcs.meshName)
        #end
        
        
        ax.set_ylabel('Surface Area (mm^2)')
        plt.show()
    #end
    
    def PlotSurfaceAreaRatios(mcs):
        """
        Outputs a boxplot of the Mesh surface areas divided by the true surface area

        """
        fig = plt.figure(figsize =(10, 7))
        ax = fig.add_subplot(111)
        ax.boxplot(mcs.surfaceAreaRatios);
        
        if(mcs.meshName is None):
            ax.set_title("Surface Areas ratio'd over the true Surface Area")
        else:
            ax.set_title("Surface Areas  of " + mcs.meshName + " ratio'd over the true Surface Area of " + mcs.meshName);
        ax.set_ylabel('Surface Area ratio (no unit)')
        plt.show()
    #end
    
    
    
    def __GetSurfaceAreas(mcs) -> np.ndarray:
        """
        Get Surface Areas of MeshContainers

        Returns
        -------
        np.ndarray
            array of all surface areas in mm^2.

        """
        if(mcs.__surfaceAreasHidden is None):
            SA = [];
            for mc in mcs.__MeshContainerList:
                SA.append(mc.surfaceArea);
            #end
            mcs.__surfaceAreasHidden = np.asarray(SA)
        #end
        return mcs.__surfaceAreasHidden;
    #end
    
    def __GetSurfaceAreaQuartiles(mcs) -> np.ndarray:
        """
        Get Surface Area Quartiles of MeshContainers

        Returns
        -------
        np.ndarray
            array of the quartiles of surface areas, in Q1, Q2 (average), Q3 order

        """
        if(mcs.__surfaceAreaQuartilesHidden is None):
            mcs.__surfaceAreaQuartilesHidden = F.StatsGet123_Quartiles(mcs.surfaceAreas)
        #end
        return mcs.__surfaceAreaQuartilesHidden;
    #end
    
    def __GetSurfaceAreaRatios(mcs) -> np.ndarray:
        """
        Get the ratios of the calculated surface are to the true one

        Returns
        -------
        np.ndarray
            array of all surface areas / true surface area. Unitless

        """
        
        if mcs.trueSurfaceArea is None:
            raise Exception("No True Surface Area defined!")
        #end
        
        if(mcs.__SurfaceAreaRatiosHidden is None):
            mcs.__SurfaceAreaRatiosHidden = mcs.surfaceAreas / mcs.trueSurfaceArea;
        #end
        return mcs.__SurfaceAreaRatiosHidden;
    #end
    
    def __GetSurfaceAreaQuartileRatios(mcs) -> np.ndarray:
        """
        Gets the Quartils of the ratios of the surface areas

        Returns
        -------
        np.ndarray
            array of the quartiles of surface area ratios, in Q1, Q2 (average), Q3 order

        """
        if(mcs.__surfaceAreaQuartilesHidden is None):
            mcs.__surfaceAreaQuartilesHidden = F.StatsGet123_Quartiles(mcs.surfaceAreaRatios);
        #end
        return mcs.__surfaceAreaQuartilesHidden;
    #end
        
    
    def __GetNumMeshContainers(mcs) -> int:
        return len(mcs.__MeshContainerList);
    
    def __ResetCachedValues(mcs):
        mcs.__surfaceAreasHidden = None;
        mcs.__surfaceAreaQuartilesHidden = None;
        mcs.__SurfaceAreaRatiosHidden = None;
        mcs.__SurfaceAreaQuartilesRatiosHidden = None;
    #end
    
    @property
    def MeshContainers(mcs) -> list['MeshContainer']:
        return mcs.__MeshContainerList;
    #end
    
    @MeshContainers.setter
    def UpdateMeshContainers(mcs, value : list['MeshContainer']):
        mcs.__MeshContainerList = value;
        mcs.__ResetCachedValues();
    #end
    
    @property
    def trueSurfaceArea(mcs) -> float:
        """
        The actual surface area these geometries are representing

        Returns
        -------
        float
            surface area in mm^2.

        """
        if(mcs.__trueSurfaceAreaHidden is None):
            raise Exception("No True Surface area has been set!");
        #end
        return mcs.__trueSurfaceAreaHidden;
    #end
    
    @trueSurfaceArea.setter
    def trueSurfaceArea(mcs, value: float):
        mcs.__trueSurfaceAreaHidden = value;
        mcs.__ResetCachedValues();
    #end
    
    size = property(fget=__GetNumMeshContainers)
    surfaceAreas = property(fget=__GetSurfaceAreas)
    surfaceAreaQuartiles = property(fget=__GetSurfaceAreaQuartiles)
    surfaceAreaRatios = property(fget=__GetSurfaceAreaRatios)
    surfaceAreaRatiosQuartiles = property(fget=__GetSurfaceAreaQuartileRatios)