import open3d as o3d
import numpy as np
from PickedPointsSet import PickedPointSet
import string
from skimage.color import rgb2hsv
import HelperFunctions as F
import math
import random
import copy
from typing import Tuple


class MeshContainer:
    """
    Holds Single TriangleMesh data
    Also holds functions relevant to the mesh

    """

    def __init__(mc, mesh: o3d.geometry.TriangleMesh or str, debug : bool = False, cleanOnImport : bool = True, linearCalibration : float = 1.145):
        
        mc.mesh : o3d.geometry.TriangleMesh;
        
        if(isinstance(mesh, o3d.geometry.TriangleMesh)):
            mc.mesh = mesh
        elif(isinstance(mesh, str)):
            RawMesh  = o3d.io.read_triangle_mesh(mesh)
            if(cleanOnImport):
                RawMesh = mc.__CleanMesh(RawMesh)
            # end
            mc.mesh = RawMesh

        mc.debug : bool = debug
        mc.PickedPointsSets : dict[str, 'PickedPointSet'] = {}
        mc.__linearCalibration : float = linearCalibration
        
        mc.__ResetStored();
    # end

    def VisualizeMesh(mc):
        """
        Visualizes mesh.
        Program paused until user closes window

        """
        o3d.visualization.draw_geometries([mc.mesh])
    # end

    def VisualizeNewCoordinatesWithMesh(mc, coords: np.ndarray):
        o3d.visualization.draw_geometries(
            [F.GeneratePointCloudFromCoords(coords), mc.mesh])
    # end

    def GetPickedPoints(mc, pickedPointSetName: str, numPoints: int, ShouldForceNew : bool = False) -> PickedPointSet:
        """
        Returns a pickedPointSet Object of the name PickedPointSet from the MeshContainer. If one does not exist it will be created with user input

        Parameters
        ----------
        pickedPointSetName : string
            The name of the PickedPointSet to grab or generate
        numPoints : int
            Number of points to generate if we are generating a PickedPointSet. Defaults to the metamodels "DefaultNumberPointsPerPickedPointsSet"
        ShouldForceNew: bool, optional (default False)
            Should force new creation of named point set despite one of the same name already existing
        Returns
        -------
        PickedPointSetClass
            The PickedPointSet, which holds information and methods regarding PickedPoints

        """
        if not ShouldForceNew:
            if pickedPointSetName in mc.PickedPointsSets.keys():
                # set already exists
                return mc.PickedPointsSets[pickedPointSetName]
            # end
        # end

        print("No points of set " + str(pickedPointSetName) +
              " detected. Please pick " + str(numPoints) + " points.")

        # initialize User UI to pick points
        PickedPointsNum = 0
        vis = o3d.visualization.VisualizerWithVertexSelection()
        vis.create_window()
        vis.add_geometry(mc.mesh)
        while PickedPointsNum < numPoints:
            PickedPointsNum = len(vis.get_picked_points())
            vis.poll_events()
        vis.destroy_window()

        # intialize PickedPointSet object and store
        pickedPoints = vis.get_picked_points()
        pickedPointIndexes = np.zeros(len(pickedPoints), dtype=int)
        for i, pickedPoint in enumerate(pickedPoints):
            pickedPointIndexes[i] = pickedPoint.index
        # end
        mc.PickedPointsSets[pickedPointSetName] = PickedPointSet(
            pickedPointIndexes, mc.mesh, mc)
        return mc.PickedPointsSets[pickedPointSetName]
    # end

    def SetPickedPoints(mc, pickedPointSetName: str, pointIndexes: np.ndarray) -> PickedPointSet:
        """
        Define a PickedPointSet by directly specifying point indexes (IE no user interaction)

        Parameters
        ----------
        pickedPointSetName : string
            name of the PickedPointSet to be stored into the MeshContainer.
        pointIndexes : np.ndarray
            1D int array of point indexes

        Returns
        -------
        PickedPointSet
            PickedPointSet.

        """

        mc.PickedPointsSets[pickedPointSetName] = PickedPointSet(
            pointIndexes, mc.mesh, mc)
        return mc.PickedPointsSets[pickedPointSetName]
    # end

    def FindDepth(mc, planePointsName: str = None, depthPointName: str = None) -> float:
        """
        Calculates a plane given 3 points of planePointsName and one point depthPointName, and then finds the distance between them.

        Parameters
        ----------
        planePointsName : str, optional
            if given, will use this named PickedPointSet of 3 points for the plane
        depthPointName : str, optional
            if given, will use this named PickedPointSet of 1 point for the depth point

        Returns
        -------
        float
            the depth in mm.

        """

        def DistFromPointToPlane(PlaneL, PlaneR, p):
            return (abs((PlaneL[0] * p[0]) + (PlaneL[1] * p[1]) + (PlaneL[2] * p[2]) + PlaneR)) / math.sqrt((PlaneL[0]**2) + (PlaneL[1]**2) + (PlaneL[2]**2))
        # end

        ForceNew = False
        if(planePointsName is None):
            planePointsName = "PlanePoints"
            depthPointName = "DepthPoint"
            ForceNew = True
        # end

        planePoints = mc.GetPickedPoints(
            planePointsName, 3, ShouldForceNew=ForceNew)
        depthPoint = mc.GetPickedPoints(
            depthPointName, 1, ShouldForceNew=ForceNew)

        P1 = planePoints.coordinates[0]
        P2 = planePoints.coordinates[1]
        P3 = planePoints.coordinates[2]

        P0 = depthPoint.coordinates[0]

        V12 = P2 - P1
        V13 = P3 - P1

        PlaneL = np.cross(V12, V13)
        PlaneR = (PlaneL[0] * -P1[0]) - \
            (PlaneL[1] * P1[1]) - (PlaneL[2] * P1[2])

        dist = DistFromPointToPlane(PlaneL, PlaneR, P0) * mc.__linearCalibration
        return dist
    # end

    def GetDistanceBetweenPoints(mc, numberOfPoints : int = 2, pickedPointSetPerimeterName: str = None, close : bool = False) -> float:
        """
        Returns the distance between 2 or more points in sequential order (like a perimeter)

        Parameters
        ----------
        numberOfPoints : int, optional
            Number of points to get if no pickedPointSet was given. The default is 2.
        pickedPointSetPerimeterName : string, optional
            PickedPointSet to use for perimeter if you wish to define a predefined one
        close: bool ,optional
            if True, will also take the distance between the first and last point like a perimeter. Default is False

        Returns
        -------
        float
            distance between points in sequential order (mm).

        """

        def DistBetween2Points(pA, pB) -> float:
            return np.sqrt(((pA[0] - pB[0])**2) + ((pA[1] - pB[1])**2) + ((pA[2] - pB[2])**2))
        # end

        if(numberOfPoints < 2):
            print("Cannot measure distances with less than 2 points!")
            return 0.0
        # end

        force = False
        if(pickedPointSetPerimeterName == None):
            force = True
            pickedPointSetPerimeterName = "defaultPerimeterSet"
        # end

        points = mc.GetPickedPoints(
            pickedPointSetPerimeterName, numberOfPoints, ShouldForceNew=force)

        dist = 0.0

        for i in range(numberOfPoints - 1):
            dist += DistBetween2Points(
                points.coordinates[i], points.coordinates[(i + 1)])
        # end

        if(close):
            dist += DistBetween2Points(
                points.coordinates[0], points.coordinates[(numberOfPoints - 1)])
        # end

        return dist * mc.__linearCalibration
    # end

    def VisualizeSelectedPoints(mc, selectedPointIndexesSets: list['PickedPointSet']):
        """
        Highlights selected PointSets in a visualization

        Parameters
        ----------
        selectedPointIndexesSets : list
            str list of PickedPointSets you wish to visualize.

        Returns
        -------
        None.

        """

        numDim = len(selectedPointIndexesSets)
        modelsToShow = []
        for i in range(numDim):
            relevantCoords = mc.PickedPointsSets[selectedPointIndexesSets[i]].coordinates
            R = random.randint(0, 255)
            G = random.randint(0, 255)
            B = round(255 - ((R + G) / 2))
            PC = F.GeneratePointCloudFromCoords(
                relevantCoords, colorRGB=np.asarray([R, G, B]))
            modelsToShow.append(PC)
        # end

        modelsToShow.append(mc.mesh)

        o3d.visualization.draw_geometries(modelsToShow)
    # end

    def Wizard(mc) -> 'MeshContainer':
        '''Easy to Use Menu for running commands and generating automations'''
        def QueryUser() -> str:
            '''Retrieves next step from user'''
            print("Please type in the next action you wish to complete. Case matters!")
            print("Type 'Help' to see the list of available options.\n")
            return input()

        def GetMC(meshContainerHistory: list, currentStep: int) -> MeshContainer:
            return meshContainerHistory[currentStep]

        def DecisionTree(choice : str, currentStep: int,meshContainerHistory: list,commandHistory: list) -> list[bool, int, list, list]:

            def GetColorChannel() -> str:
                print("Please type in a color channel to use. Cases matter!")
                print("Options: red green blue hue saturation value\n")
                options: list = ["red", "green", "blue", "hue", "saturation", "value"]
                while (True):
                    inputValue = input()
                    if (inputValue in options):
                        return inputValue
                    # end
                    print("Invalid option selected. Try again: \n")
                # end

            def Previous(currentStep: int, commandHistory: list):
                if(currentStep == 0):
                    print("You did not run any commands")
                    return
                print(f'The last command you ran was {commandHistory[currentStep - 1]["Command"]}')

            def UndoPrevStep(currentStep: int, commandHistory: list, meshContainerHistory: list) -> list[
                int, list, list]:

                commandHistory = commandHistory[:-1]
                meshContainerHistory = meshContainerHistory[:-1]
                currentStep = currentStep - 1
                return [currentStep, commandHistory, meshContainerHistory]

            def PrintHelp():
                '''Prints Help Statement for User'''
                print(
                    "VisualizeMesh : Shows the current mesh in a window. Execution will not continue until it's closed.")
                print(
                    "FindDepth : Allows you to pick a plane and then a point to measure the depth/height of structures.")
                print("GetDistanceBetweenPoints : allows you to find the distance between 2 points you pick.")
                print("CropByBoundingBox : Allows you to do a manual hard-crop within 4 picked points.")
                print("RemoveCompletelyBlackPoints : Removed Points that were likely erroneously colored.")
                print(
                    "DivideByOtsuColorSinglePickedPoint : Pick color channel to Otsu's segment with, then pick a side.")
                print("DivideGiven2PPS : Pick an inside and outside region and the system will attempt to segment it.")
                print("Previous : What was the last command you ran?")
                print("Undo : Undos your previous command")
                print("Stop : stops the wizard, outputs the current mesh and steps taken to get there.\n")

            def VM(currentStep: int, meshContainerHistory: list):
                hmc: MeshContainer = GetMC(meshContainerHistory, currentStep)
                hmc.VisualizeMesh()

            def FD(currentStep: int, meshContainerHistory: list) -> list[dict, MeshContainer]:
                hmc: MeshContainer = GetMC(meshContainerHistory, currentStep)
                numDist: float = hmc.FindDepth()
                print(f"Measured Depth was: {numDist}")

                PlanePoints: np.ndarray = hmc.PickedPointsSets["PlanePoints"].indexes
                DepthPoint: np.ndarray = hmc.PickedPointsSets["DepthPoint"].indexes

                dOut: dict = {
                    "Command": "FindDepth",
                    "PlanePoints": PlanePoints,
                    "DepthPoint": DepthPoint
                }

                return [dOut, hmc]

            def GDBP(currentStep: int, meshContainerHistory: list) -> list[dict, MeshContainer]:
                hmc: MeshContainer = GetMC(meshContainerHistory, currentStep)
                numDist: float = hmc.GetDistanceBetweenPoints()
                print(f"Measured Distance was: {numDist}")

                perimeterPoints: np.ndarray = hmc.PickedPointsSets["defaultPerimeterSet"].indexes

                dOut: dict = {
                    "Command": "GetDistanceBetweenPoints",
                    "perimeterPoints": (perimeterPoints)
                }

                return [dOut, hmc]

            def CBBB(currentStep: int, meshContainerHistory: list) -> list[dict, MeshContainer]:
                hmc: MeshContainer = GetMC(meshContainerHistory, currentStep)

                newMC: MeshContainer
                newMC = hmc.CropByBoundingBox()

                PP: np.ndarray = hmc.PickedPointsSets["Points To Crop"].indexes

                dOut: dict = {
                    "Command": "CropByBoundingBox",
                    "Points To Crop": (PP)
                }

                return [dOut, newMC]

            def RCBP(currentStep: int, meshContainerHistory: list) -> list[dict, MeshContainer]:
                hmc: MeshContainer = GetMC(meshContainerHistory, currentStep)

                newMC: MeshContainer

                newMC , _ = hmc.RemoveCompletelyBlackPoints()

                dOut: dict = {"Command": "RemoveCompletelyBlackPoints"}
                return [dOut, newMC]

            def DBOCSPP(currentStep: int, meshContainerHistory: list, colorChannel: str) -> list[dict, MeshContainer]:
                hmc: MeshContainer = GetMC(meshContainerHistory, currentStep)

                newMC: MeshContainer
                newMC, _ = hmc.DivideByOtsuColorSinglePickedPoint(colorChannel)

                PP: np.ndarray = hmc.PickedPointsSets["_SinglePoint"].indexes

                dOut: dict = {
                    "Command": "DivideByOtsuColorSinglePickedPoint",
                    "colorChannel": colorChannel,
                    "_SinglePoint": (PP)
                }

                return [dOut, newMC]

            def DG2PPS(currentStep: int, meshContainerHistory: list) -> list[dict, MeshContainer]:
                hmc: MeshContainer = GetMC(meshContainerHistory, currentStep)

                newMC: MeshContainer
                newMC, _ = hmc.DivideGiven2PPS(numberAdjacentSearches=2)

                PPin: np.ndarray = hmc.PickedPointsSets["_InsidePoints"].indexes
                PPout: np.ndarray = hmc.PickedPointsSets["_OutsidePoints"].indexes

                dOut: dict = {
                    "Command": "DivideGiven2PPS",
                    "_InsidePoints": (PPin),
                    "_OutsidePoints": (PPout)
                }

                return [dOut, newMC]

            # This mess is why C# is better
            if choice == "Help":
                PrintHelp()
                return [False, currentStep, meshContainerHistory, commandHistory]
            if choice == "VisualizeMesh":
                VM(currentStep, meshContainerHistory)
                return [False, currentStep, meshContainerHistory, commandHistory]
            if choice == "Previous":
                Previous(currentStep, commandHistory)
                return [False, currentStep, meshContainerHistory, commandHistory]
            if choice == "Undo":
                if (currentStep == 0):
                    print("There are no previous steps to undo!")
                    return [False, currentStep, commandHistory, meshContainerHistory]
                currentStep, commandHistory, meshContainerHistory = UndoPrevStep(currentStep, commandHistory, meshContainerHistory)
                return [False, currentStep, meshContainerHistory, commandHistory]
            if choice == "Stop":
                return [True, currentStep, meshContainerHistory, commandHistory]

            # Mesh Modifiers

            newMesh: MeshContainer
            dOut: dict
            unknown: bool = True

            if choice == "FindDepth":
                dOut, newMesh = FD(currentStep, meshContainerHistory)
                unknown = False
            if choice == "GetDistanceBetweenPoints":
                dOut, newMesh = GDBP(currentStep, meshContainerHistory)
                unknown = False
            if choice == "CropByBoundingBox":
                dOut, newMesh = CBBB(currentStep, meshContainerHistory)
                unknown = False
            if choice == "RemoveCompletelyBlackPoints":
                dOut, newMesh = RCBP(currentStep, meshContainerHistory)
                unknown = False
            if choice == "DivideByOtsuColorSinglePickedPoint":
                choice: str = GetColorChannel()
                dOut, newMesh = DBOCSPP(currentStep, meshContainerHistory, choice)
                unknown = False
            if choice == "DivideGiven2PPS":
                dOut, newMesh = DG2PPS(currentStep, meshContainerHistory)
                unknown = False

            if unknown:
                print("Invalid option, try again")
                return [False, currentStep, meshContainerHistory, commandHistory]


            meshContainerHistory.append(newMesh)
            commandHistory.append(dOut)
            currentStep = currentStep + 1

            return [False, currentStep, meshContainerHistory, commandHistory]

        def OutputHistory(commandHistory: list):

            if len(commandHistory) == 0:
                return "No Steps taken"

            output: str = "dictionary: dict = ["
            for dictCommand in commandHistory:
                output = output + F.dict2String(dictCommand) + ","
            # end
            output = output[:-1] + "]"
            return output


        # CODE STARTS HERE

        print("Welcome to the Segmentation Helper Wizard!")

        currentStep : int = 0
        meshContainerHistory: list = [copy.copy(mc)]
        commandHistory: list = [] # Elements are dicts with key for the command, and other keys for other relevant info
        isLookFinished: bool = False

        # Ask the user what to do, build up command history

        while not isLookFinished:
            userResponse: str = QueryUser()
            isLookFinished, currentStep, meshContainerHistory, commandHistory = DecisionTree(userResponse, currentStep, meshContainerHistory, commandHistory)
        # end

        print("Process Complete! Printing out the steps into the console. Copy and paste it to get a sequence of commands to reuse.\n\n")
        print(OutputHistory(commandHistory))
        return GetMC(meshContainerHistory, currentStep)
    # end

    def WizardLoad(mc, commands: list) -> 'MeshContainer':

        def DecisionTree(inputDict: dict, inputMC: MeshContainer) -> MeshContainer:

            command: str = inputDict["Command"]

            if command == "FindDepth":
                inputMC.SetPickedPoints("PlanePoints", inputDict["PlanePoints"])
                inputMC.SetPickedPoints("DepthPoint", inputDict["DepthPoint"])
                inputMC.FindDepth(planePointsName="PlanePoints", depthPointName="DepthPoint")
                return inputMC
            if command == "GetDistanceBetweenPoints":
                inputMC.SetPickedPoints("perimeterPoints", inputDict["perimeterPoints"])
                inputMC.GetDistanceBetweenPoints(pickedPointSetPerimeterName="perimeterPoints")
                return inputMC
            if command == "CropByBoundingBox":
                inputMC.SetPickedPoints("Points To Crop", inputDict["Points To Crop"])
                inputMC = inputMC.CropByBoundingBox(boundingBoxInitialPPSName="Points To Crop")
                return inputMC
            if command == "RemoveCompletelyBlackPoints":
                inputMC, _ = inputMC.RemoveCompletelyBlackPoints()
                return inputMC
            if command == "DivideByOtsuColorSinglePickedPoint":
                inputMC.SetPickedPoints("_SinglePoint", inputDict["_SinglePoint"])
                inputMC, _ = inputMC.DivideByOtsuColorSinglePickedPoint(inputDict["colorChannel"], singlePickedPointSetName= "_SinglePoint")
                return inputMC


            # DivideGiven2PPS
            inputMC.SetPickedPoints("_InsidePoints", inputDict["_InsidePoints"])
            inputMC.SetPickedPoints("_OutsidePoints", inputDict["_OutsidePoints"])
            inputMC, _ = inputMC.DivideGiven2PPS(PPSInsideName="_InsidePoints", PPSOutsideName="_OutsidePoints", numberAdjacentSearches=2)
            return inputMC
        # end

        output: MeshContainer = copy.copy(mc)
        for element in commands:
            output = DecisionTree(element, output)

        return output
    # end


    def __ResetStored(mc):
        """
        Resets non-mesh related properties of the object incase recalculations need to be done.
        (Or for initialization)

        Returns
        -------
        None.

        """
        mc.__RGBStats_STORED = None
        mc.__HSVStats_STORED = None
        mc.__triangles_RGB_STORED = None
        mc.PickedPointsSets = {}
        mc.__edgeSetArr = None
    # end

    def __CleanMesh(mc, triangleMesh: o3d.geometry.TriangleMesh) -> o3d.geometry.TriangleMesh:
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
        triangleMesh : o3d.geometry.TriangleMesh = o3d.geometry.TriangleMesh.remove_duplicated_vertices(
            triangleMesh)
        triangleMesh = o3d.geometry.TriangleMesh.remove_degenerate_triangles(
            triangleMesh)
        return o3d.geometry.TriangleMesh.remove_unreferenced_vertices(triangleMesh)
    # end

    def __GetNumberVerts(mc) -> int:
        """
        Returns number of vertices in the mesh

        Returns
        -------
        int
            number of vertices.

        """
        return np.asarray(mc.mesh.vertices).shape[0]
    # end

    def __GetCoordinates(mc) -> np.ndarray:
        """
        Returns np array of coordinates of mesh

        Returns
        -------
        np.ndarray
            array of point coordinates.

        """
        return np.asarray(mc.mesh.vertices)
    # end

    def __GetRGB(mc) -> np.ndarray:
        """
        Returns RGB array of all points in mesh

        Returns
        -------
        np.ndarray
            RGB array of all points in mesh.

        """

        return np.asarray(mc.mesh.vertex_colors)
    # end

    def __GetRGBStats(mc) -> list:
        """
        returns quartile 1, 2, 3 for each color channel.

        Returns
        -------
        list
            quartile arrays (one per RGB channel).

        """
        if(mc.__RGBStats_STORED is None):
            output = []
            for i in range(3):  # yes this is non-pythonic, I don't care
                output.append(F.StatsGet123_Quartiles(mc.RGB[i]))
            # end
            mc.__RGBStats_STORED = output
        # end
        return mc.__RGBStats_STORED
    # end

    def __GetHSV(mc) -> np.ndarray:
        """
        returns HSV colors for every point in mesh

        Returns
        -------
        np.ndarray
            HSV colors for every point in mesh.

        """
        return rgb2hsv(mc.RGB)
    # end

    def __GetHSVStats(mc) -> list:
        """
        returns quartile 1, 2, 3 for each color channel.

        Returns
        -------
        list
            quartile arrays (one per HSV channel).

        """
        if(mc.__HSVStats_STORED is None):
            mc.__HSVStats_STORED = F.StatsGet123_Quartiles(mc.HSV)
        # end
        return mc.__HSVStats_STORED
    # end
    
    def __GetRGBHSVStats(mc) -> list:
        return (mc.RGB + mc.HSV);

    def __GetColorChannelByIndex(mc, index: int) -> np.ndarray:
        """
        Returns single channel color array for mesh (either RGB or HSV) based on index

        Parameters
        ----------
        index : int
            0-2 for R G B and 3-5 for H S V.

        Returns
        -------
        colorChannel : 1D np.ndarray
            color channel float array.

        """
        if(index < 3):
            return mc.RGB[:, index]
        else:
            return mc.HSV[:, index - 3]
        # end
    # end

    def __GetColorChannelByString(mc, name: str) -> np.ndarray:
        """
        Gets Single Color channel by string name

        Parameters
        ----------
        name : string
            Name of channel to grab.

        Returns
        -------
    1D np.ndarray
        Color channel data.
        """

        return mc.__GetColorChannelByIndex(F.ColorStringToInt(name))
    # end

    def __GetTriangles(mc) -> np.ndarray:
        """
        Returns Triangle array from mesh

        Returns
        -------
        np.ndarray
            Triangle array from mesh.

        """
        return np.asarray(mc.mesh.triangles)
    # end

    def __GetTrianglesRGB(mc) -> np.ndarray:
        """
        generates array of RGB value of each triangle (produced by averaging colors of each of triangles 3 verts)

        Returns
        -------
        np.ndarray
            array of RGB colors per triangle of dimensions (numTriangles x 3)

        """
        if(mc.__triangles_RGB_STORED is None):
            numTriangles = len(mc.triangles)
            trianglesRGB = np.zeros((numTriangles, 3))
            vertsRGB = mc.RGB

            for numIndexTriangle in range(numTriangles):
                Vert1, Vert2, Vert3 = mc.triangles[numIndexTriangle]
                trianglesRGB[numIndexTriangle] = (
                    vertsRGB[Vert1] + vertsRGB[Vert2] + vertsRGB[Vert3]) / 3
            # end
            mc.__triangles_RGB_STORED = trianglesRGB
        # end
        return mc.__triangles_RGB_STORED
    # end

    def __GetCenterCoordinateOfMesh(mc) -> np.ndarray:
        """
        returns center coordinate of mesh

        Returns
        -------
        np.ndarray
            center coordinate of mesh.

        """
        return mc.mesh.get_center()
    # end

    def __GetSurfaceArea(mc) -> float:
        """
        returns surface area of mesh itself


        Returns
        -------
        float
            area in mm^2.

        """
        calibration = mc.__linearCalibration ** 2  # offset via calibration value (squared since this is an area and the calibration is linear)
        return mc.mesh.get_surface_area() * calibration

    def __ComputeConvexHull(mc):
        return mc.mesh.compute_convex_hull()
    # end

    def __ContainerFromMesh(mc, mesh: o3d.geometry.TriangleMesh) -> 'MeshContainer':
        return MeshContainer(mesh, linearCalibration = mc.__linearCalibration, debug = mc.debug);
    #end
    
    def __GetColorChannelDataFromUserSelection(mc, colorChannel : str or int) -> np.ndarray:
        """
        Given a user request for channel data, returns the correct color channel

        Parameters
        ----------
        colorChannel : str or int
            color channel string name or index (red green blue hue saturation value) numbered 0-5.

        Returns
        -------
        np.ndarray
            1D array of color data in mesh.

        """
        #mfw when no switch statement
        if(isinstance(colorChannel, str)):
            colorChannelData = mc.__GetColorChannelByString(colorChannel);
        elif(isinstance(colorChannel, int)):
            colorChannelData = mc.__GetColorChannelByIndex(colorChannel);
        else:
            raise Exception(" 'colorChannel' must be either a string or an int!");
        #end
        return colorChannelData;
    #end
    
    #Todo : sets could probably be used here instead of lists for optimization reasons
    def __GetAllPointsIncludingAndAdjacent(mc, indexes : np.ndarray) -> np.ndarray:
        """
        Given a PickedPointSet, returns an array of all point indexes including the picked point set and adjacent points to it.

        Parameters
        ----------
        indexes : np.ndarray
            int array of indexes of whose points to check for adjacents

        Returns
        -------
        np.ndarray
            int array of picked points and adjacent points

        """
        
        if not mc.mesh.has_adjacency_list():
            mc.mesh = mc.mesh.compute_adjacency_list();
        #end
        
        output : list = indexes.tolist();
        numPoints : int = len(output);
        
        for i in range(numPoints):
            adjPoints = mc.mesh.adjacency_list[indexes[i]];
            output = output + list(adjPoints);
        #end
        
        output = [*set(output)]; #hacky way to remove duplicates froma  list
        return np.asarray(output);
    #end
    
    def __GetAllAdjacentPointsMultiLayer(mc, startingIndexes : np.ndarray, numTimes : int) -> np.ndarray:
        """
        Gets all points adjacent to the given point indexes, then the points adjacent to those, and so on the number of times defined.
        THIS ALGORITHM GETS EXPONENTIALLY SLOWER WITH EVERY ITERATION INCREASE!

        Parameters
        ----------
        startingIndexes : np.ndarray
            starting point indices to start searching.
        numTimes : int
            number of times to get adjacent. Highly suggested to keep this less than 4

        Returns
        -------
        np.ndarray
            int array of point indices, no repeats.

        """
        
        
        # warn against stupidity
        if(numTimes > 4):
            print("WARNING: You selected a large number of adjacent search iterations. This algorithm is not efficient so this will take a while...")
        #end
        
        output : np.ndarray = startingIndexes;
        
        for i in range(numTimes):
            output = mc.__GetAllPointsIncludingAndAdjacent(output);
        #end
        
        return output;
    #end
    
    def Save(mc, folderPath: str, fileName: str):
        """allows for saving of meshContainers as ply files"""
        o3d.io.write_triangle_mesh((folderPath + fileName + ".ply", mc.mesh))
    # end

    @property
    def linearCalibration(mc) -> float:
        """
        A property to return the linear calibration. 
        Used due to the custom setter function


        Returns
        -------
        float
            linear calibration currently set on this meshContainer.

        """
        return mc._MeshContainer__linearCalibration
    
    @linearCalibration.setter
    def linearCalibration(mc, value : float):
        """
        Resets certain calculated values that will be impacted by having a new calibration value

        Parameters
        ----------
        value : float
            the new calibration value.

        """
        mc._MeshContainer__linearCalibration = value;
        mc._MeshContainer__ResetStored();
    #end
    
    '''
    # TODO STILL IN PROGRESS - NON FUNCTIONAL
    def GetNonManifoldVerts(mc) -> list:
        """
        Generates an list of arrays of vertices on the edge of a model.
        Will show visualization if mc.debug is true

        Returns
        -------
        list
            np.ndarrays of vertex points on the edges of models, grouped by edge group.

        """

        if(mc.__edgeSetArr is None):
            # if there is no adjaceny list, calculate it
            if(mc.mesh.has_adjacency_list() == False):
                mc.mesh.compute_adjacency_list()
            # end

            # find the (first) index of a specific value from an array
            def GetEdgeArrayIndexFromVertIndex(element: int, searchingArray: np.ndarray) -> int:
                return np.flatnonzero(searchingArray == element)[0]
            # end

            # find all edge verts connecting to an edge vert
            def GetConnectedEdgeVerts(vert: int, allEdgeVertsSet: np.ndarray) -> np.ndarray:
                adj = np.asarray(mc.mesh.adjacency_list[vert])
                return np.intersect1d(adj, allEdgeVertsSet, assume_unique=True)
            # end

            # TODO is this kinda akward, fix this
            # Checks if starting vert is not branching, if so returns the next vert (direction random), else returns false
            def GetNextVertFromStartingVert(firstVert: int, CurrentVertScanningArr: np.ndarray) -> list:
                connectedVerts = GetConnectedEdgeVerts(
                    firstVert, CurrentVertScanningArr)
                if(connectedVerts.size == 2):
                    # we are on a straight path, pick one random direction
                    return [True, connectedVerts[0]]
                # end
                #vert is branching
                return [False, -1]  # doesnt matter what vert return
            # end

            # given there is a past vert, and the path is not branching, find the next vert.

            def GetNextVertGivenStraightPath(adjacencyArr: np.ndarray, prevVert: int) -> int:
                return np.delete(adjacencyArr, prevVert)[0]
            # end

            # generate vars
            AllEdgeVertsNP = np.unique((np.asarray(mc.mesh.get_non_manifold_edges(
                allow_boundary_edges=False))).flatten())  # cursed
            AllEdgeVertsGroupID = np.ones(AllEdgeVertsNP.size) * -1
            AllEdgeVertsNPIndexesRemaining = range(AllEdgeVertsNP.size)
            GroupID = 0
            isFirstElementInGroup = True

            # iterate along all Edge Verts
            for currentAllVertIndex in AllEdgeVertsNPIndexesRemaining:

                # Skip if the current vertex is already in an edge group
                if(AllEdgeVertsGroupID[currentAllVertIndex]):
                    continue
                # end

                currentAllVert = AllEdgeVertsNP[currentAllVertIndex]

            # TODO clean mesh of extra stuff first
        # end
    # end
    '''
    
    
    ###########################################################################
    ########################## COLOR SEGMENTANTATION ##########################
    ###########################################################################

    #TODO - BoundingBox is flawed, points don't always end up quite where expected
    def CropByBoundingBox(mc, boundingBoxInitialPPSName : str = None, cubeHalfOffsetHeight : float = 0.1, previewCrop : bool = False) -> 'MeshContainer':
        """
        Using a set of 4 points, projects them off the surface in both directions to create 8 points to make a bounding box, then crops with said box

        Parameters
        ----------
        boundingBoxInitialPPSName : str, optional
            if given, will use this PPS of 4 points instead of asking the user
        cubeHalfOffsetHeight : float, optional
            distance in meters for the points to offset from each Picked Point each way. The default is 0.1.
        previewCrop : bool, optional
            if set to True, will visualize crop for user to see before completing the function. Defaults to False

        Returns
        -------
        MeshContainer
           Mesh Container consisting of mesh inside the bounding box.

        """


        # TODO Flawed
        def FindBoundingBoxFrom4Points(arrPoints : np.ndarray, halfCubeHeight : float) -> Tuple[o3d.geometry.AxisAlignedBoundingBox, np.ndarray]:

            def SelectIndexesBesidesSelf(indSearching : int, numIndexes : int) -> np.ndarray:
                '''creates an array of indexes, and removes a specific element from it'''
                arrIndexes = range(numIndexes)
                return np.delete(arrIndexes, indSearching)
            # end

            def VectorBetween2PointsFromArray(arrPoints : np.ndarray, searchingPoint : int) -> np.ndarray:
                '''find the vector between 2 points along array,skipping self'''
                V3Array = np.zeros((3, 3))
                pointsToCompare = SelectIndexesBesidesSelf(searchingPoint, 4)
                p1 = arrPoints[searchingPoint]
                i = 0
                for pointIndex in pointsToCompare:
                    p2 = arrPoints[pointIndex]
                    V3Array[i] = p2 - p1
                    i = i + 1
                # end
                return V3Array
            # end

            def RemoveLongestVectorFromArray(arr: np.ndarray) -> np.ndarray:
                '''takes in array of vectors, returns array of vectors with the longest removed'''
                maxDist = -1
                maxIndex = -1
                numElements = len(arr)
                for index in range(numElements):
                    D = np.linalg.norm(arr[index])
                    if D > maxDist:
                        maxDist = D
                        maxIndex = index
                    # end
                # end
                return np.delete(arr, maxIndex, 0)
            # end

            def ConvertToUnitVector(V : np.ndarray) -> np.ndarray:
                '''coverts to unit vector'''
                V = V / (np.linalg.norm(V))
                return V
            # end


            cubePoints = np.zeros((8, 3))
            i = 0

            for pointIndex in range(4):
                curPointDistFromOthers = VectorBetween2PointsFromArray(
                    arrPoints, pointIndex)
                curPointDistFromOthers = RemoveLongestVectorFromArray(
                    curPointDistFromOthers)
                crossCurrentPoint = np.cross(
                    curPointDistFromOthers[0], curPointDistFromOthers[1])
                Vector = ConvertToUnitVector(
                    crossCurrentPoint) * halfCubeHeight
                cubePoints[i] = arrPoints[pointIndex] + Vector
                i = i + 1
                cubePoints[i] = arrPoints[pointIndex] - Vector
                i = i + 1
            # end

            BoundingBox = o3d.geometry.AxisAlignedBoundingBox.create_from_points(
                o3d.utility.Vector3dVector(cubePoints))

            return BoundingBox, cubePoints
        # end
        
        forceNew = False;
        if(boundingBoxInitialPPSName is None):
            boundingBoxInitialPPSName = "Points To Crop";
            forceNew = True;
        #end

        pointsToCropFrom = mc.GetPickedPoints(boundingBoxInitialPPSName, numPoints=4, ShouldForceNew = forceNew);
        coordsToCropFrom = pointsToCropFrom.coordinates;
        BoundingBox, CubePoints = FindBoundingBoxFrom4Points(coordsToCropFrom, cubeHalfOffsetHeight)
        newMesh = mc.mesh.crop(BoundingBox)
        

        
        if(previewCrop):
            o3d.visualization.draw_geometries([newMesh]);
        #end
        
        return mc.__ContainerFromMesh(newMesh);
    # end

    def RemoveIslands(mc, minimumArea: float = 50) ->'MeshContainer':
        multiOutput: tuple = mc.mesh.cluster_connected_triangles()
        triangleIndeces: np.ndarray = np.asarray(multiOutput[0])
        triangleAreas: np.ndarray = np.asarray(multiOutput[2])

        indexTriangleTooSmall: np.ndarray = np.argwhere(triangleAreas < minimumArea)
        triangleMask: np.ndarray = np.isin(triangleIndeces, indexTriangleTooSmall) # true if too small
        newMesh: o3d.geometry.TriangleMesh = copy.copy(mc.mesh)
        newMesh.remove_triangles_by_mask(triangleMask)
        newMesh = newMesh.remove_unreferenced_vertices()
        newMesh = newMesh.remove_degenerate_triangles()
        return MeshContainer(newMesh)



    def RemoveCompletelyBlackPoints(mc,maxBrightness : float = 0.01) -> list['MeshContainer', 'MeshContainer']:
        """
        Used To output a mesh with completely black points removed. 
        These points are created when mesh data is captured but the camera misses color data. 
        Since they impact color segmentation, they should be removed

        Parameters
        ----------
        maxBrightness : float, optional
            max brightness for a point before it is no longer cut off. The default is 0.01.

        Returns
        -------
        list['MeshContainer', 'MeshContainer']
            a passed and rejected MeshContainer in that order.

        """
        #breakpoint();
        return mc.DivideByColorBandFilter(-0.01, maxBrightness, False, "value");
    #end
    
    def DivideByColorBandFilter(mc, bottomBound : float, topBound : float, isbandPass : bool, colorChannel : str or int) -> list['MeshContainer', 'MeshContainer']:
        """
        returns a pass and reject MeshContainer resultance of a band pass/ reject filter based on a specified RGB/HSV color

        Parameters
        ----------
        bottomBound : float
            0-1 lower bound for the band filter.
        topBound : float
            0-1 upper bound for the band filter.
        isbandPass : bool
            if this a band pass filter? else band reject.
        colorChannel : str or int
            (red green blue hue saturation value) or numbers 0-5 to pick the color channel to apply the color band filter to.


        Returns
        -------
        list['MeshContainer', 'MeshContainer']
            a passed and rejected MeshContainer in that order.

        """
        
        if(bottomBound > topBound):
            raise Exception('Bottom bound cannot be greater than top bound');
        #end
        
        meshPass, meshReject = mc.__DivideMeshByColorBandFilter(isbandPass, np.asarray([topBound, bottomBound]), colorChannel);
        
        return mc.__ContainerFromMesh(meshPass), mc.__ContainerFromMesh(meshReject);
    #end

    def DivideGiven2PPS(mc, PPSInsideName: str = "_InsidePoints", PPSOutsideName: str = "_OutsidePoints", numNewPoints: int = 4, weights : np.ndarray = np.asarray([1,1,1,1]), numberAdjacentSearches : int = 0) -> list['MeshContainer', 'MeshContainer']:
        """
        returns 2 MeshContainers based on 2 regions of picked points

        Parameters
        ----------
        PPSInsideName : str, optional
            if defined, allows you to prespecify the inner region (PPS) of what the segment (desired). Else user is asked.
        PPSOutsideName : str, optional
            if defined, allows you to prespecify the outer region (PPS) of what the segment (undesired). Else user is asked.
        numNewPoints : int, optional
            if user is to pick the inner/outer points, number of points each set gets. Defaults to 4.
        weights : np.ndarray, optional
            For scoring, scale the importance of Shared area, Inverse Median Distance, Inner IQR and Outer IQR in that order. The smallest score is the winner among all color channels. The default is np.asarray([1,1,1,1]).
        includeAdjacents : bool, optional
            Also include points adjacent to all selected picked points. This gives a better range of data to work with. Defaults to False.
            
        Returns
        -------
        list
            inner (desired) meshContainer, outer (undesired) meshContainer

        """
        

        # if no PPS is given, just force create new ones from user
        shouldForceNew = (PPSInsideName == "_InsidePoints")
        pickedPointsDesired = mc.GetPickedPoints(
            PPSInsideName, numNewPoints, ShouldForceNew=shouldForceNew)
        pickedPointsUndesired = mc.GetPickedPoints(
            PPSOutsideName, numNewPoints, ShouldForceNew=shouldForceNew)
        
        if(numberAdjacentSearches > 0):
            pickedPointsDesired = mc.SetPickedPoints(PPSInsideName + "_Adj", mc.__GetAllAdjacentPointsMultiLayer(pickedPointsDesired.indexes, numberAdjacentSearches));
            pickedPointsUndesired = mc.SetPickedPoints(PPSOutsideName + "_Adj", mc.__GetAllAdjacentPointsMultiLayer(pickedPointsUndesired.indexes, numberAdjacentSearches));
        #end
        
        bestColorIndex : int = F.ColorStringToInt(mc.__SelectBestColorGiven2PPS(pickedPointsDesired, pickedPointsUndesired, weights = weights))
        
        bestChannelData : np.ndarray = np.concatenate((pickedPointsDesired.GetColorChannelByInt(bestColorIndex), pickedPointsUndesired.GetColorChannelByInt(bestColorIndex)));
        
        averageDesired : float = pickedPointsDesired.GetColorStatsByInt(bestColorIndex)[1];
        
        passMesh, rejectMesh = mc.__CustomOtsuFilter(bestChannelData, averageDesired, bestColorIndex);
        return mc.__ContainerFromMesh(passMesh), mc.__ContainerFromMesh(rejectMesh);
    #end
            
    def DivideByOtsuColor(mc, colorChannel : str or int, valToPass : float = 1.0) -> list['MeshContainer', 'MeshContainer']:
        """
        Divides a mesh into 2 over a specified color channel via the Otsus method

        Parameters
        ----------
        colorChannel : str or int
            (red green blue hue saturation value) or numbers 0-5 to pick the color channel divide
        valToPass : float, optional
            The value that you want the passed side to contain. The default is 1.0.

        Returns
        -------
        list['MeshContainer', 'MeshContainer']
            a passed and rejected MeshContainer in that order.

        """
        
        colorChannelData : np.ndarray = mc.__GetColorChannelDataFromUserSelection(colorChannel);
        otsuBoundry : float = mc.__OtsuThreshold(colorChannelData);
        
        passBounds : np.ndarray = np.asarray([1.0, otsuBoundry]);
        isValToPassInBand : bool = (valToPass > otsuBoundry);
        
        passMesh : o3d.geometry.TriangleMesh;
        rejectMesh : o3d.geometry.TriangleMesh;
        
        passMesh, rejectMesh = mc.__DivideMeshByColorBandFilter(isValToPassInBand, passBounds, colorChannel);
        return mc.__ContainerFromMesh(passMesh), mc.__ContainerFromMesh(rejectMesh);
    #end
    
    def DivideByOtsuColorSinglePickedPoint(mc, colorChannel : str or int, singlePickedPointSetName : str = "_SinglePoint") -> list['MeshContainer', 'MeshContainer']:
        """
        Divides a mesh into 2 over a specified color channel via the Otsus method. side picked chosen by a single point either predefined or picked by user

        Parameters
        ----------
        colorChannel : str or int
            (red green blue hue saturation value) or numbers 0-5 to pick the color channel divide
        singlePickedPointSetName : str, optional
            if defined, lets you use a existing PPS to pick what side to keep, otherwise user is asked. The default is "_SinglePoint".

        Returns
        -------
        list['MeshContainer', 'MeshContainer']
            a passed and rejected MeshContainer in that order.

        """
        
        forceNew : bool = (singlePickedPointSetName == "_SinglePoint")
        singlePointSet : PickedPointSet = mc.GetPickedPoints(singlePickedPointSetName, 1, ShouldForceNew = forceNew);
        
        if(isinstance(colorChannel, str)):
            colorChannel = F.ColorStringToInt(colorChannel);
        #end
        
        valToPick : float = singlePointSet.RGBHSV_Quartiles[colorChannel][1]; #get average
        return mc.DivideByOtsuColor(colorChannel, valToPick);
        

    def __DivideMeshViaMask(mc, mask: np.ndarray) -> list:
        """
        Returns a list of 2 meshes, seperated by the input boolean array (vertex mask)

        Parameters
        ----------
        mask : np.ndarray
            boolean np array, 1D of length of number of verts in parent mesh.


        Returns
        -------
        list
            contains 2 meshes, where the first contains all points where mask is true, and the next mesh the rest.

        """

        meshA = copy.deepcopy(mc.mesh)
        meshB = copy.deepcopy(mc.mesh)
        meshB.remove_vertices_by_mask(mask)
        meshA.remove_vertices_by_mask(np.invert(mask))
        return meshA, meshB
    # end

    def __SSD(mc, counts, centers):
        """ Sum of squared deviations from mean """
        n = np.sum(counts)
        mu = np.sum(centers * counts) / n
        return np.sum(counts * ((centers - mu) ** 2))
    # end 
    
    def __OtsuThreshold(mc, singleChannleArr : np.ndarray) -> float:
        """
        Given a single channel of data, finds the otsu threshold to divide it evenly

        Parameters
        ----------
        singleChannleArr : np.ndarray
            single channel (1D) data.

        Returns
        -------
        float
            threshold.

        """
        # credit: https://bic-berkeley.github.io/psych-214-fall-2016/otsu_threshold.html
        numBins = 256  # this number possible values per single channel pixel
        counts, edges = np.histogram(singleChannleArr[:], bins=numBins)
        bin_centers = edges[:-1] + np.diff(edges) / 2
        total_ssds = []
        #breakpoint()
        for bin_no in range(1, numBins):
            left_ssd = mc._MeshContainer__SSD(counts[:bin_no], bin_centers[:bin_no])
            right_ssd = mc._MeshContainer__SSD(counts[bin_no:], bin_centers[bin_no:])
            total_ssds.append(left_ssd + right_ssd)
        z = np.argmin(total_ssds)
        #t = bin_centers[z];
        print('Otsu threshold (c[z]):', bin_centers[z])
        return bin_centers[z]
    # end
    
    
    def __DivideMeshByColorBandFilter(mc, isPass: bool, boundsArr: np.ndarray, colorChannel : int or str) -> list[o3d.geometry.TriangleMesh, o3d.geometry.TriangleMesh]:
        """
        BandPass/block filter. uses masks for a specific color channel based on bound input, returning 2 meshes

        Parameters
        ----------
        isPass : bool
            is this a bandpass filter? if not the it is a band block.
        boundsArr : np.ndarray
            contains upper and lower bound.
        colorChannel : int or str
            the color channel to Divide

        Returns
        -------
        meshA : o3d.geometry.TriangleMesh
            Passed region.
        meshB : o3d.geometry.TriangleMesh
            rejected region.

        """
        colorChannelData = mc.__GetColorChannelDataFromUserSelection(colorChannel);
        
        if(isPass):
            maskUpper = (colorChannelData < boundsArr[0])
            maskLower = (colorChannelData > boundsArr[1])
            mask = maskLower & maskUpper
        else:
            maskUpper = (colorChannelData > boundsArr[0])
            maskLower = (colorChannelData < boundsArr[1])
            mask = maskLower | maskUpper     
        #end

        [Pass, Reject] = mc.__DivideMeshViaMask(mask)
        return Pass, Reject
    # end

    def __CustomOtsuFilter(mc, channelData: np.ndarray, valueOfSideToKeep: float, colorChannel : str or int) -> list[o3d.geometry.TriangleMesh, o3d.geometry.TriangleMesh]:
        """
        Given Channel Data, the otsu threshold value is calculated. Then, the side side of the 
        channel where ValueOfSideToKeep is bandpassed is passed and vice versa using ColorBandFilter function

        Parameters
        ----------
        channelData : np.ndarray
            custom Channel Data.
        valueOfSideToKeep : float
            what side of the threshold should be kept in the bandpass.
        colorChannel : int or str
            the Channel that will be processed on

        Returns
        -------
        list
            list of triangle meshes.

        """

        otsuThreshold = mc.__OtsuThreshold(channelData)

        # Make sure the correct side is passed
        if valueOfSideToKeep == None:
            boundsArr = [0, otsuThreshold]
        else:
            if valueOfSideToKeep < otsuThreshold:
                boundsArr = [otsuThreshold, 0.0]
            else:
                boundsArr = [1.0, otsuThreshold]
            # end
        # end

        return mc._MeshContainer__DivideMeshByColorBandFilter(True, boundsArr, colorChannel); # this is terrible, but pythons inability consistently to do inheritance forced this upon us
    # end
    
    def __SelectBestColorGiven2PPS(mc, DesiredPPS : PickedPointSet, UnDesiredPPS: PickedPointSet, weights : np.ndarray = np.asarray([1,0.5,1,1])) -> str:
        """
        Given 2 Picked Point Sets, finds the best color channel that can seperate them

        Parameters
        ----------
        DesiredPPS : PickedPointSet
            The Picked Point Set on the side you want to keep.
        UnDesiredPPS : PickedPointSet
            The Picked point Set on the side you want to crop out.
        weights : np.ndarray, optional
            For scoring, scale the importance of Shared area, Inverse Median Distance, Inner IQR and Outer IQR in that order. The smallest score is the winner among all color channels. The default is np.asarray([1,0.5,1,1]).

        Returns
        -------
        str
            string name of the color channel to use.

        """
        
        if(mc.debug):
            print("selecting best channel for otsu segmentation given 2 PickedPointSets")
        #end
        
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

            HistInside = np.histogram(
                InsidePoints * 255, bins=np.arange(0, 256))
            HistOutside = np.histogram(
                OutsidePoints * 255, bins=np.arange(0, 256))

            SharedArea = 0.0;

            # sum up intersecting areas per bin
            for x in range(255):
                isInnerPresent = HistInside[0][x] > 0
                isOuterPresent = HistOutside[0][x] > 0
                if(isInnerPresent & isOuterPresent):
                    SharedArea += min(HistInside[0][x], HistOutside[0][x])
                # end
            # end

            # normalize
            SharedArea = SharedArea / len(InsidePoints)

            return SharedArea
        # end
        
        def InvDistanceBetweenMedians(ChannelStatsInside: np.ndarray, ChannelStatesOutside: np.ndarray) -> float:
            """
            Calculates the inverse distance between 2 medians of 2 given channels

            Parameters
            ----------
            ChannelStatsInside : np.ndarray
                desired side color Channel stat (Q1, Q2, Q3) data.
            ChannelStatesOutside : np.ndarray
                Undesired side color Channel (Q1, Q2, Q3) stat data.

            Returns
            -------
            float
                Inverse distance between medians.
            """
            
            return 1 - abs(ChannelStatsInside[1] - ChannelStatesOutside[1])
        # end
        
        RGBHSV_Scores = np.zeros((4, 6))
        
        for colorIndex in range(6):
            
            #Get color specific data
            DesiredColorChannel : np.ndarray = DesiredPPS.GetColorChannelByInt(colorIndex);
            UnDesiredColorChannel : np.ndarray = UnDesiredPPS.GetColorChannelByInt(colorIndex);
            DesiredStats : np.ndarray = DesiredPPS.GetColorStatsByInt(colorIndex);
            UnDesiredStats : np.ndarray = UnDesiredPPS.GetColorStatsByInt(colorIndex);
           
            #Get Scores
            SharedArea : float = SharedAreaBetweenPoints(DesiredColorChannel, UnDesiredColorChannel);
            InvDistBtwnMedians : float = InvDistanceBetweenMedians(DesiredStats, UnDesiredStats);
            IQR_In : float = DesiredStats[2] - DesiredStats[0];
            IQR_Out : float = UnDesiredStats[2] - UnDesiredStats[0];
            
            if(mc.debug):
                print("Analyzing color channel " + F.IntToColorString(colorIndex));
                print("Shared area: " + str(SharedArea))
                print("Inverse Distance Between Medians: " + str(InvDistBtwnMedians))
                print("IQR inside: " + str(IQR_In))
                print("IQR Outside: " + str(IQR_Out))
            #end
            
            
            
            
            
            #save scores (keeping this seperate instead of merging with the top for easier readability / understanding)
            RGBHSV_Scores[0, colorIndex] = SharedArea * weights[0]
            RGBHSV_Scores[1, colorIndex] = InvDistBtwnMedians * weights[1]
            RGBHSV_Scores[2, colorIndex] = IQR_In * weights[2]
            RGBHSV_Scores[3, colorIndex] = IQR_Out * weights[3]
        #end
        
        # sum and pick best channel
        sums : np.ndarray = RGBHSV_Scores.sum(axis=0)
        bestChannelIndex : int = np.arange(6)[np.argmin(sums)]
        bestChannelName : str = F.IntToColorString(bestChannelIndex);
        
        
        
        if(mc.debug):
            print("Best Channel is" + bestChannelName)
        # end
        
        return bestChannelName;
    #end  


    vertices = property(fget=__GetCoordinates)
    numVerts = property(fget=__GetNumberVerts)
    RGB = property(fget=__GetRGB)
    HSV = property(fget=__GetHSV)
    RGB_Quartiles = property(fget=__GetRGBStats)
    HSV_Quartiles = property(fget=__GetHSVStats)
    RGBHSV_Quartiles = property(fget = __GetRGBHSVStats)
    triangles = property(fget=__GetTriangles)
    triangles_RGB = property(fget=__GetTrianglesRGB)
    surfaceArea = property(fget=__GetSurfaceArea)
    #edgeVertArrs = property(fget=GetNonManifoldVerts)
# end
