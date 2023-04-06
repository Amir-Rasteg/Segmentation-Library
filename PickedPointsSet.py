import open3d as o3d
import numpy as np
from skimage.color import rgb2hsv
import HelperFunctions as F


class PickedPointSet:
    """
    Container to hold a set of user (or given) 'picked' points. These points typically picked from the UI typically
    only contain the vertex index. This class extends that to easily retrieve color and statistical information from
    those points
    """

    # noinspection PyTypeChecker
    def __init__(pp, pickedPointIndexes: np.ndarray, mesh: o3d.geometry.TriangleMesh, self):

        pp.indexes: np.ndarray = pickedPointIndexes
        pp.coordinates: np.ndarray = np.asarray(mesh.vertices)[pickedPointIndexes]
        pp.RGB: np.ndarray = np.asarray(mesh.vertex_colors)[pickedPointIndexes]
        pp.pointCount: int = len(pickedPointIndexes)

        pp.__HSV_STORED: np.ndarray = None
        pp.__RGBHSV_STORED: np.ndarray = None
        pp.__coordinatesQuartiles_STORED: np.ndarray = None
        pp.__RGBQuartiles_STORED: np.ndarray = None
        pp.__HSVQuartiles_STORED: np.ndarray = None
        pp.self = self
    
    def __GetHSV(pp) -> np.ndarray:
        """
        Returns HSV Array in order of point indexes

        Returns
        -------
        TYPE np.ndarray of dimensions [(number of verts) x 3].

        """
        if pp.__HSV_STORED is None:
            pp.__HSV_STORED = np.asarray(rgb2hsv(pp.RGB))
        return pp.__HSV_STORED
    
    def __GetRGBQuartiles(pp) -> np.ndarray:
        """
        returns quartile 1, 2, 3 for each color channel

        Returns
        -------
        2D array
        R_Q1, R_Q2, R_Q3
        G_Q1...

        """
        if pp.__RGBQuartiles_STORED is None:
            R: np.ndarray = F.StatsGet123_Quartiles(pp.RGB[:, 0])
            G: np.ndarray = F.StatsGet123_Quartiles(pp.RGB[:, 1])
            B: np.ndarray = F.StatsGet123_Quartiles(pp.RGB[:, 2])
            pp.__RGBQuartiles_STORED = np.hstack((R, G, B))
        return pp.__RGBQuartiles_STORED

    def __GetRGBHSV(pp) -> np.ndarray:
        if pp.__RGBHSV_STORED is None:
            pp.__RGBHSV_STORED = np.vstack(pp.RGB + pp.HSV)
        return pp.__RGBHSV_STORED

    def __GetHSVQuartiles(pp) -> np.ndarray:
        """
        returns quartile 1, 2, 3 for each color channel

        Returns
        -------
        2D array
        H_Q1, H_Q2, H_Q3
        S_Q1...

        """
        if pp.__HSVQuartiles_STORED is None:
            H: np.ndarray = F.StatsGet123_Quartiles(pp.HSV[:, 0])
            S: np.ndarray = F.StatsGet123_Quartiles(pp.HSV[:, 1])
            V: np.ndarray = F.StatsGet123_Quartiles(pp.HSV[:, 2])
            pp.__HSVQuartiles_STORED = np.hstack((H, S, V))
        return pp.__HSVQuartiles_STORED
    
    def __GetRGBHSVQuartiles(pp) -> np.ndarray:
        """
        returns Quartiles 1, 2, 3 for every color channel
        Returns
        -------
        2D array
        R_Q1, R_Q2, R_Q3
        G_Q1...
        B_Q1...
        H_Q1...
        """
        return np.hstack((pp.RGB_Quartiles, pp.HSV_Quartiles))  # since inputs are always small, no need to cache

    def __GetCoordQuartiles(pp) -> np.ndarray:
        """
        returns quartile 1, 2, 3 for each XYZ channel in stored coordinate

        Returns
        -------
        2D array
        X_Q1, X_Q2, X_Q3
        Y_Q1...

        """
        if pp.__coordinatesQuartiles_STORED is None:
            X: np.ndarray = F.StatsGet123_Quartiles((pp.coordinates[:, 0]))
            Y: np.ndarray = F.StatsGet123_Quartiles((pp.coordinates[:, 1]))
            Z: np.ndarray = F.StatsGet123_Quartiles((pp.coordinates[:, 2]))

            pp.__coordinatesQuartiles_STORED = np.hstack((X, Y, Z))
        return pp.__coordinatesQuartiles_STORED
    
    def GetColorChannelByInt(pp, channel: int or str) -> np.ndarray:
        """
        Returns a specific color channel by int index

        Parameters
        ----------
        channel : int or str
            0-2 for RGB, 3-5 for HSV. Channel string name can be used instead as well

        Returns
        -------
        Color Channel: np.ndarray
            1D color channel.

        """
        if isinstance(channel, str):
            channel = F.ColorStringToInt(channel)
        if not isinstance(channel, int):
            raise Exception("Non-valid type for color channel")
        
        if channel < 3:
            return pp.RGB[:, channel]
        else:
            return pp.HSV[:, channel-3]
    
    def GetColorStatsByInt(pp, channel: int or str) -> np.ndarray:
        """
        Returns a specific color statistics np.ndarray by int index

        Parameters
        ----------
        channel : int
            0-2 for RGB, 3-5 for HSV. Channel string name can be used instead as well

        Returns
        -------
        Color stats: np.ndarray
            Q1 Q2 Q3 stats for a channel.

        """
        if isinstance(channel, str):
            channel = F.ColorStringToInt(channel)
        if not isinstance(channel, int):
            raise Exception("Non-valid type for color channel")
        
        if channel < 3:
            return pp.RGB_Quartiles[channel]
        else:
            return pp.HSV_Quartiles[channel - 3]
    
    HSV = property(fget=__GetHSV)
    RGBHSV = property(fget=__GetRGBHSV)
    coordinate_Quartiles = property(fget=__GetCoordQuartiles)
    RGB_Quartiles = property(fget=__GetRGBQuartiles)
    HSV_Quartiles = property(fget=__GetHSVQuartiles)
    RGBHSV_Quartiles = property(fget=__GetRGBHSVQuartiles)
