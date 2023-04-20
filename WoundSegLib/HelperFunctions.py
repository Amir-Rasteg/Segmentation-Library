# Functions for use in scripts
import numpy as np
import open3d as o3d


def ColorStringToInt(name: str) -> int:
    """
    Given string of color channel, returns indexing int 0-5 for RGBHSV

    Parameters
    ----------
    name : string
        string name of channel.

    Returns
    -------
    int
        index of channel.

    """
    try:
        possibleStrings = ['red', 'green', 'blue', 'hue', 'saturation', 'value']
        index = possibleStrings.index(name.lower())
        return index
    except ValueError as error:
        raise Exception(f"input string {name} is not an accepted color")


def IntToColorString(index: int) -> str:
    """
    Given the integer index of all possible color channels, returns the color channel name as a string

    Parameters
    ----------
    index : int
        index of the channel.

    Returns
    -------
    str
        string name of the channel.

    """
    if index < 0:
        raise Exception("Cannot have a negative index")
    if index > 5:
        raise Exception("Color index cannot be greater than 5")
    possibleStrings = ['red', 'green', 'blue', 'hue', 'saturation', 'value']
    return possibleStrings[index]

def IntOrStringToColorInt(index) -> int:
    if isinstance(index, str):
        index = ColorStringToInt(index)
    if index > 5:
        raise Exception("Color index cannot be greater than 5")
    return index

def IntOrStringToColorString(index) -> int:
    if isinstance(index, int):
        index = IntToColorString(index)
    if index not in ['red', 'green', 'blue', 'hue', 'saturation', 'value']:
        raise Exception(f"input string {index} is not an accepted color")
    return index


def StatsGet123_Quartiles(arrayIn: np.ndarray) -> np.ndarray:
    """
    Takes in a single dimension np.array and returns an np.array of the 1st, 2nd, and 3rd quartiles

    Parameters
    ----------
    arrayIn : np.ndarray (any length 1D)

    Returns
    -------
    TYPE : np.ndarray (1D length of 3)
    """
    output = np.zeros(3)
    output[0] = np.quantile(arrayIn, 0.25)
    output[1] = np.quantile(arrayIn, 0.50)
    output[2] = np.quantile(arrayIn, 0.75)
    return np.asarray(output)


def GeneratePointCloudFromCoords(coords: np.ndarray, colorRGB=np.asarray([0, 255, 0])) -> o3d.geometry.PointCloud:
    """
    Generates a New PointCloud given a set of XYZ coordinates and optionaly colors. No triangles

    Parameters
    ----------
    coords : np.ndarray
        XYZ coordinates.
    colorRGB : np.ndarray, optional
        RGB Color for all points to be. The default is np.asarray([0,255,0]).

    Returns
    -------
    o3d,geometry.PointCloud
        CLoud returned.

    """

    colorRGB = colorRGB / 255
    newMesh = o3d.geometry.PointCloud()
    newMesh.points = o3d.utility.Vector3dVector(coords)
    newMesh.paint_uniform_color(colorRGB)
    return newMesh


def GetEdgeVertsFromTriangleMesh(mesh: o3d.geometry.TriangleMesh) -> np.ndarray:
    """
    Returns array of Edge Vert Indices from Mesh

    Parameters
    ----------
    mesh : o3d.geometry.TriangleMesh
        Triangle Mesh Input.

    Returns
    -------
    np.ndarray
        1D int array of point indexes.

    """
    return np.unique((np.asarray(mesh.get_non_manifold_edges(allow_boundary_edges=False))).flatten())  # cursed


def PointListToString(points: list) -> str:
    """turns a list of numpy arrays into a string"""
    
    def NPArrToString(arr: np.ndarray) -> str:
        """turns a numpy array into a string that you can input to code"""
        outString: str = "np.asarray(["
        for number in arr:
            outString = outString + str(number) + ","
        outString = outString.rstrip(outString[-1])
        outString = outString + "])"
        return outString
        
    output: str = "["
    for element in points:
        output = output + NPArrToString(element) + ","
    output = output.rstrip(output[-1])
    output = output + "]"
    return output


def NumpyArray2String(array: np.ndarray) -> str:

    string = "np.asarray(["
    for element in array:
        string = string + str(element) + ","
    string = string[:-1] + "])"
    return string


def dict2String(dictionary: dict) -> str:

    string = "{"
    for key in dictionary.keys():
        substring: str = "'" + str(key) + "': "
        if isinstance(dictionary[key], str):
            substring = substring + "'" + dictionary[key] + "', "
        else:
            # assume numpy array
            substring = substring + NumpyArray2String(dictionary[key])

        string = string + substring + ","

    string = string[:-1] + "}"
    return string
