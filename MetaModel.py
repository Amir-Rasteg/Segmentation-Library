import open3d as o3d;
import numpy as np;
from skimage.color import rgb2hsv;
import copy;
import random
import string
import HelperFunctions as F;
import time;
import math;
from MeshContainer import MeshContainer;

def TriangleMeshes2Segments(triangleMeshes: o3d.geometry.TriangleMesh) -> list:
    """
    Converts plain meshes to MetaModels

    Parameters
    ----------
    triangleMeshes : o3d.geometry.TriangleMesh
        list of triangleMeshes.

    Returns
    -------
    list
    list of MetaModels.

    """
            
            
    segments = [];
    for meshIndex in range(len(triangleMeshes)):

        temp = MetaModel(triangleMeshes[meshIndex]);
        segments.append(temp);
    #end
    return segments;
#end

class MetaModel:
    '''Master Data holder'''
    
    def __init__(self, meshesIn : dict, showMeshProcessing = False):
        
        def ImportRawMeshFromPath(path : str) -> MeshContainer:
            """
            Given a file path, imports the model and cleans it up

            Parameters
            ----------
            path : str
                path to model.

            Returns
            -------
            MeshContainer
                Cleaned up model.

            """
            RawMesh = o3d.io.read_triangle_mesh(path);
            return MeshContainer(self.CleanMesh(RawMesh));
        #end
        
        self.showMeshProcessing = showMeshProcessing;
        self.containers = {};
        
        for key in meshesIn.keys():
            element = meshesIn[key];
            
            if(isinstance(element, str)):
                #if input is a string, assume it is a path
                self.containers[key] = ImportRawMeshFromPath(element);
                continue;
            #end
            
            if(isinstance(element, o3d.geometry.TriangleMesh)):
                self.containers[key] = MeshContainer(element);
                continue;
            #end
            
            if(isinstance(element, MeshContainer)):
                self.containers[key] = element;
                continue;
            #end
            
            #if we end up, an insupported data type was passed
            raise TypeError(key + " is not of supported type string (path), o3d.geometry.TriangleMesh, or MeshContainer!");
        #end
        

    #end
       
    def CleanMesh(self, triangleMesh : o3d.geometry.TriangleMesh) -> o3d.geometry.TriangleMesh:
        """
        Cleans artifacts meshes tend to have after directly being scanned, mainly duplicate verts

        Parameters
        ----------
        triangleMesh : o3d.geometry.TriangleMesh
            Mesh to be cleaned.

        Returns
        -------
        o3d.geometry.TriangleMesh
            cleaned mesh.

        """
        triangleMesh = o3d.geometry.TriangleMesh.remove_duplicated_vertices(triangleMesh);
        triangleMesh = o3d.geometry.TriangleMesh.remove_degenerate_triangles(triangleMesh);
        return o3d.geometry.TriangleMesh.remove_unreferenced_vertices(triangleMesh);
    #end  
        
    def VisualizeMesh(self, ContainerNames):
        """
        Opens a window to let you see the model in 3D. Pauses execution until closed

        Parameters
        ----------
        ContainerNames : list, optional
            list of names (string) of Containers you wish to visualize

        Returns
        -------
        None.

        """
        meshes = []
        for s in ContainerNames:
            meshes.append(self.containers[s].mesh);
        #end
        o3d.visualization.draw_geometries(meshes);
    #end
    
    def __VisualizeNewCoordinateswithMesh(self, coords : np.ndarray, mesh: o3d.geometry.TriangleMesh):
        o3d.visualization.draw_geometries([F.GeneratePointCloudFromCoords(coords), mesh]) ;
    #end
    
    def __GetDecimatedQuadMesh(self, iterations, mesh) -> o3d.geometry.TriangleMesh:
        """
        returns the decimated triangle mesh. Does not change the object itself.
        Uses smooth_laplacian Algorithm

        Parameters
        ----------
        iterations : int
            Number of times to run algorithm.

        Returns
        -------
        TYPE
            triangle mesh.

        """
        return mesh.filter_smooth_taubin();
    
    #TODO double check units and description
    def DuboisBodySurfaceArea(self, weight: float, height: float) -> float:
        """
        Uses the Dubois Formula to aproximate the body surface area

        Parameters
        ----------
        weight : float
            weight of the body in Kg.
        height : float
            height of the person in millieters.

        Returns
        -------
        float
            surface area in ??????.

        """
        return (weight ** 0.425) * ((height / 1000.0) ** 0.725) * 0.007184;
    
    #TODO double check units
    def GetSurfaceAreaRatio(self, weight: float, height: float, ContainerName) -> float:
        """
        estimates ratio of given surface area to the total body surface area using the Dubois formula

        Parameters
        ----------
        weight : float
            weight of entire patient in kg.
        height : float
            height of the patient in meters.

        Returns
        -------
        float of Surface Area Ratio.

        """
        BodySurfaceArea =  self.DuboisBodySurfaceArea(weight, height);
        return (self.containers[ContainerName].surfaceArea) / (BodySurfaceArea * 100 * 100);
    #end
    
    
    #TODO - expand / fix
    def __GetVolumeOLD(self, method = "hull") -> float:
        if(self.__volume_STORED is not None):
            return self.__volume_STORED;
        #end
        if(method == "hull"):
            self.__volume_STORED = self.mainMesh.get_volume();
            return self.__volume_STORED;
        #end
    #end
    
    ###########################################################################
    ############################# SEGMENTANTATION #############################
    ###########################################################################
    
    def __DivideContainerViaMask(self, mask : np.ndarray, containerName : str) -> list:
        """
        Returns a list of 2 meshes, seperated by the input boolean array (vertex mask)

        Parameters
        ----------
        mask : np.ndarray
            boolean np array, 1D of length of number of verts in parent mesh.
        containerName : str
            name of container to split (will not be modified)


        Returns
        -------
        list
            contains 2 meshes, where the first contains all points where mask is true, and the next mesh the rest.

        """
        
        meshA = copy.deepcopy(self.containers[containerName].mesh);
        meshB = copy.deepcopy(self.containers[containerName].mesh);
        meshB.remove_vertices_by_mask(mask);
        meshA.remove_vertices_by_mask( np.invert(mask) );
        return meshA, meshB;
    #end    
    
    def __SSD(counts, centers):
        """ Sum of squared deviations from mean """
        n = np.sum(counts);
        mu = np.sum(centers * counts) / n;
        return np.sum(counts * ((centers - mu) ** 2));
    #end
    
    def __CreateMaskByColorBandFilter(self, isPass: bool, boundsArr: np.ndarray, colorChannelData: np.ndarray) -> list:
        """
        BandPass/block filter. creates masks for a specific color channel based on bound input and applies, returning 2 meshes

        Parameters
        ----------
        isPass : bool
            is this a bandpass filter? if not the it is a band block.
        boundsArr : np.ndarray
            contains upper and lower bound.
        colorChannel : np.ndarray
            color channel data.

        Returns
        -------
        meshA : o3d.geometry.TriangleMesh
            Passed region.
        meshB : o3d.geometry.TriangleMesh
            rejected region.

        """
        maskUpper = ((colorChannelData < boundsArr[0]) == isPass)
        maskLower = ((colorChannelData > boundsArr[1]) == isPass)
        mask = maskLower | maskUpper;
        [Pass, Reject] = self.__DivideContainerViaMask(mask, False);
        return Pass, Reject;
    #end
    
    def __OtsuFilter(self, channelData: np.ndarray, valueOfSideToKeep: float) -> list:
        """
        Given color Channel Data, the otsu threshold value is calculated. Then, the side side of the 
        channel where ValueOfSideToKeep is bandpassed is passed and vice versa using ColorBandFilter function

        Parameters
        ----------
        channelData : np.ndarray
            color channel data.
        valueOfSideToKeep : float
            what side of the threshold should be kept in the bandpass.

        Returns
        -------
        list
            list of triangle meshes.

        """
        
        def Otsu(self, arr):
            #credit: https://bic-berkeley.github.io/psych-214-fall-2016/otsu_threshold.html
            numBins = 256; #this number possible values per single channel pixel
            counts, edges = np.histogram(arr[:], bins=numBins);
            bin_centers = edges[:-1] + np.diff(edges) / 2;
            total_ssds = [];
            for bin_no in range(1, numBins):
                left_ssd = self.__SSD(counts[:bin_no], bin_centers[:bin_no]);
                right_ssd = self.__SSD(counts[bin_no:], bin_centers[bin_no:]);
                total_ssds.append(left_ssd + right_ssd);
            z = np.argmin(total_ssds);
            #t = bin_centers[z];
            print('Otsu threshold (c[z]):', bin_centers[z]);
            return bin_centers[z];
        #end
           
    
        otsuThreshold = Otsu(channelData)
        
        #Make sure the correct side is passed
        if valueOfSideToKeep == None:
            boundsArr = [0, otsuThreshold];
        else:
            if valueOfSideToKeep < otsuThreshold:
                boundsArr = [otsuThreshold, 1.0];
            else:
                boundsArr = [0.0, otsuThreshold];
            #end
        #end
        
        return self.__CreateMaskByColorBandFilter(True, boundsArr, colorChannelData);
    #end
    
    def __SelectBestColorChannelGivenPPS(self, containerName : str, PPSInsideName : str = None, PPSOutsideName : str = None, numNewPoints : int = 4 ) -> str:
        """
        returns string of best color channel to segment with given 2 sets of picked points

        Parameters
        ----------
        containerName : str
            container mesh.
        PPSInsideName : str, optional
            if defined, allows you to prespecify the inner region of what the segment (desired). Else user is asked
        PPSOutsideName : str, optional
            if defined, allows you to prespecify the outer region of what the segment (undesired). Else user is asked
        numNewPoints : int, optional
            if user is to pick the inner/outer points, number of points each set gets. Defaults to 4

        Returns
        -------
        str
            color channel (red green blue hue saturation value).

        """

                  
        def SharedAreaBetweenPoints(InsidePoints: np.ndarray, OutsidePoints: np.ndarray) -> float:
            """
            Finds histogram overlap between to desired / undesired regions of a color channel, returns a score

            Parameters
            ----------
            InsidePoints : ndarray
                Desired Points.
            OutsidePoints : ndarray
                Undesired Points.

            Returns
            -------
            float:
                amount of overlap.

            """
            
            HistInside = np.histogram(InsidePoints * 255, bins = np.arange(0,256));
            HistOutside = np.histogram(OutsidePoints * 255, bins = np.arange(0,256));
            
            SharedArea = 0;
            
            #sum up intersecting areas per bin
            for x in range(255):
                isInnerPresent = HistInside[0][x] > 0;
                isOuterPresent = HistOutside[0][x] > 0;
                if(isInnerPresent & isOuterPresent):
                    SharedArea += min(HistInside[0][x], HistOutside[0][x])
                #end
            #end
            
            #normalize
            SharedArea = SharedArea / len(InsidePoints);
            
            return SharedArea;
        #end
        
        def DistanceBetweenMedians(ChannelStatsInside: np.ndarray, ChannelStatesOutside: np.ndarray) -> float:
        
            """
            Calculates the distance between 2 medians of 2 given channels

            Parameters
            ----------
            ChannelStatsInside : np.ndarray
                desired side color Channel stat (Q1, Q2, Q3) data.
            ChannelStatesOutside : np.ndarray
                Undesired side color Channel (Q1, Q2, Q3) stat data.

            Returns
            -------
            float
                Distance between medians.
            """
        
            return 1 - abs(ChannelStatsInside[1] - ChannelStatesOutside[1]);
        #end
        
        
        #if no PPS is given, just force create new ones from user
        shouldForceNew = (PPSInsideName is not None)
        pickedPointsDesired = self.containers[containerName].GetPickedPoints(PPSInsideName, numNewPoints, ShouldForceNew = shouldForceNew)
        pickedPointsUndesired = self.containers[containerName].GetPickedPoints(PPSOutsideName, numNewPoints, ShouldForceNew = shouldForceNew)
        
        RGBHSV_Scores = np.zeros((4,6));
        #check per channel
        n = 0;
        
        possibleColors = ["red", "green", "blue", "hue", "saturation", "value"];
        for channel in possibleColors:
            channelInt = F.ColorStringToInt(channel);
            channelDataDesired = pickedPointsDesired.GetColorChannelByInt(channelInt);
            channelDataUndesired = pickedPointsUndesired.GetColorChannelByInt(channelInt);
            statsDesired = pickedPointsDesired.GetColorStatsByInt(channelInt);
            statsUndesired = pickedPointsUndesired.GetColorStatsByInt(channelInt);
            
            #Get Scores
            SharedArea = SharedAreaBetweenPoints(channelDataDesired, channelDataUndesired);
            DistBtwnMedians = DistanceBetweenMedians(statsDesired, statsUndesired);
            IQR_In = statsDesired[2] - statsUndesired[0];
            IQR_Out = statsDesired[2] - statsUndesired[0];
            
            RGBHSV_Scores[0,n] = SharedArea;
            RGBHSV_Scores[1,n] = DistBtwnMedians;
            RGBHSV_Scores[2,n] = IQR_In;
            RGBHSV_Scores[3,n] = IQR_Out;
            
            n = n + 1;
        #end
        
        #sum and pick best channel
        Sums = RGBHSV_Scores.sum(axis=0);
        BestChannel = np.arange(6)[np.argmin(Sums)];
        if(self.showMeshProcessing):
            print("Best Channel is", possibleColors[BestChannel]);
        #end
        return possibleColors[BestChannel];
    #end     
    
    #TODO - BoundingBox is flawed 
    def CropByBoundingBox(self, containerName : str, croppedContainerName : str, boundingBoxInitialPPSName = None) -> MeshContainer:
        
        #TODO Flawed
        def FindBoundingBoxFrom4Points(arrPoints, halfCubeHeight=None):
            
            def SelectIndexesBesidesSelf(indSearching, numIndexes):
                '''creates a list of indexes, and removes a specific element from it'''
                arrIndexes = range(numIndexes)
                return np.delete(arrIndexes, indSearching)
            #end
            
            def VectorBetween2PointsFromArray(arrPoints, searchingPoint):
                '''find the vector between 2 points along array,skipping self'''
                V3Array = np.zeros((3,3))
                pointsToCompare = SelectIndexesBesidesSelf(searchingPoint,4)
                p1 = arrPoints[searchingPoint]
                i = 0
                for pointIndex in pointsToCompare:
                    p2 = arrPoints[pointIndex]
                    V3Array[i] = p2 - p1
                    i = i + 1
                #end
                return V3Array
            #end
            
            def RemoveLongestVectorFromArray(arr):
                '''takes in array of vectors, returns array of vectors with the longest removed'''
                maxDist = -1
                maxIndex= -1
                numElements = len(arr)
                for index in range(numElements):
                    D = np.linalg.norm(arr[index])
                    if D > maxDist:
                        maxDist = D
                        maxIndex = index
                    #end
                #end
                return np.delete(arr, maxIndex, 0)
            #end

            def ConvertToUnitVector(V):
                '''coverts to unit vector'''
                V = V / (np.linalg.norm(V))
                return V
            #end
            
            if(halfCubeHeight == None):
                halfCubeHeight = 20;
            #end

            cubePoints = np.zeros((8,3))
            i = 0

            for pointIndex in range(4):
                curPointDistFromOthers = VectorBetween2PointsFromArray(arrPoints, pointIndex)
                curPointDistFromOthers = RemoveLongestVectorFromArray(curPointDistFromOthers)
                crossCurrentPoint = np.cross(curPointDistFromOthers[0],curPointDistFromOthers[1])
                Vector = ConvertToUnitVector(crossCurrentPoint) * halfCubeHeight
                cubePoints[i] = arrPoints[pointIndex] + Vector
                i = i + 1
                cubePoints[i] = arrPoints[pointIndex] - Vector
                i = i + 1
            #end

            BoundingBox = o3d.geometry.AxisAlignedBoundingBox.create_from_points(o3d.utility.Vector3dVector(cubePoints))

            return BoundingBox,cubePoints
        #end
        
        pointsToCropFrom = self.GetPickedPoints("Points To Crop", numPoints = 4);
        coordsToCropFrom = pointsToCropFrom.coordinates;
        BoundingBox, CubePoints = FindBoundingBoxFrom4Points(coordsToCropFrom)
        newMesh = copy.copy(self.containers[containerName].mesh)
        self.containers[croppedContainerName] = MeshContainer(newMesh.crop(BoundingBox));
        return self.containers[croppedContainerName];
    #end
    
    
    
    

    def Segment(self, algorithmDicts: dict) -> list:
        """
        Segmentation functions: returns an array of MetaModels segmented accoring to the input algorithmDict

        Parameters
        ----------
        algorithmDicts : dict
        keys:
            algorithm: what alg to use
            channel: red, green, blue, hue, saturation, value (brightness), auto
            (other).

        Returns
        -------
        list
            list of MetaModels segmented from self.
        """


        


        

            
        
        ############################SEGMENTING STARTS HERE############################
        
        #Variables to pass through
        colorChannelStr = None;
        colorChannelInt = None;
        colorChannelData = None;
        
        usingInvalidColors = self.containsInvalidColorPoints;
        
        algDict = algorithmDicts; #TODO - make array
        #preprocessing steps - Get PreReq data
        if "colorChannel" in algDict.keys():
            if(algDict["colorChannel"] == "auto"):
                if "numberPickedPoints" not in algDict.keys():
                    numPointsToPick = 4;
                else:
                    numPointsToPick = algDict["numberPickedPoints"]
                #end
                colorChannelStr = AUTO_PickBestChannel(numPointsToPick);
            else:
                colorChannelStr = algDict["colorChannel"];
            #end
            colorChannelInt = F.ColorStringToInt(colorChannelStr);
            colorChannelData = self.__GetColorChannelByString(colorChannelStr);
        #end
        
        
        #Now the actual filtering algorithms are here
        if(algDict["algorithm"] == "bandfilter"):
            TriangleMeshes = ColorBandFilter(algDict["isPass"], algDict["bounds"], colorChannelData);
            usingInvalidColors = False;
        #end
        if(algDict["algorithm"] == "otsu"):
            usingInvalidColors = False;
            if "sideToKeep" not in algDict.keys():
                valueOfSideToKeep = None;
            else:
                if algDict["sideToKeep"] == "lower":
                    valueOfSideToKeep = 0.0;
                if algDict["sideToKeep"] == "upper":
                    valueOfSideToKeep = 1.0;
                if algDict["sideToKeep"] == "picked":
                    if "numberPickedPoints" not in algDict.keys():
                        numPointsToPick = 4;
                    else:
                        numPointsToPick = algDict["numberPickedPoints"]
                    #end
                    PickedPointSetToUse = self.GetPickedPoints("DesiredRegion");
                    valueOfSideToKeep = PickedPointSetToUse.GetColorStatsByInt(colorChannelInt)[1];
                #end
            #end
            TriangleMeshes = OtsuFilter(colorChannelData, valueOfSideToKeep);
        #end
        
        if(algDict["algorithm"] == "hardCrop"):
            TriangleMeshes = HardCrop(usingInvalidColors);
        #end
        
        return TriangleMeshes2Segments(TriangleMeshes);
    #end
        
    

    
#end







    
