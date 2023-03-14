import open3d as o3d;
import numpy as np;
from skimage.color import rgb2hsv;
import HelperFunctions as F;


class PickedPointSet:
    '''creates picked point set objects'''
    
    def __init__(pp, pickedPointIndexes : np.ndarray, mesh : o3d.geometry.TriangleMesh, self):
        
        pp.indexes : np.ndarray = pickedPointIndexes;
        pp.coordinates : np.ndarray = np.asarray(mesh.vertices)[pickedPointIndexes];
        pp.RGB : np.ndarray = np.asarray(mesh.vertex_colors)[pickedPointIndexes]
        pp.pointCount : int = len(pickedPointIndexes);
        
        pp.__HSV_STORED : np.ndarray = None;
        pp.__coordinatesQuartiles_STORED : np.ndarray = None;
        pp.__RGBQuartiles_STORED : np.ndarray = None;
        pp.__HSVQuartiles_STORED : np.ndarray = None;
        pp.self = self;
    #end
    
    def __GetHSV(pp) -> np.ndarray:
        """
        Returns HSV Array in order of point indexes

        Returns
        -------
        TYPE np.ndarray of dimensions [(number of verts) x 3].

        """
        if(pp.__HSV_STORED is None):
            pp.__HSV_STORED = np.asarray(rgb2hsv(pp.RGB));
        #end
        return pp.__HSV_STORED;
    #end
    
    def __GetRGBQuartiles(pp) -> list[np.ndarray]:
        """
        returns quartile 1, 2, 3 for each color channel

        Returns
        -------
        list of quartile arrays (one per RGB channel)

        """
        if(pp.__RGBQuartiles_STORED is None):
            output = [[],[],[]];
            for i in range(3):
                output[i] = F.StatsGet123_Quartiles(pp.RGB[:,i]);
            #end
            pp.__RGBQuartiles_STORED = output;
        #end
        return pp.__RGBQuartiles_STORED;
    #end
            
    def __GetHSVQuartiles(pp) -> list[np.ndarray]:
        """
        returns quartile 1, 2, 3 for each color channel

        Returns
        -------
        list of quartile arrays (one per HSV channel)

        """
        if(pp.__HSVQuartiles_STORED is None):
            output = [[],[],[]];
            for i in range(3):
                output[i] = F.StatsGet123_Quartiles(pp.HSV[:,i]);
            #end
            pp.__HSVQuartiles_STORED = output;
        #end
        return pp.__HSVQuartiles_STORED;
    #end
    
    def __GetRGBHSVQuartiles(pp) -> list[np.ndarray]:
        return (pp.RGB + pp.HSV);
    
    def __GetCoordQuartiles(pp) -> list[np.ndarray]:
        """
        returns quartile 1, 2, 3 for each XYZ channel in stored coordinate

        Returns
        -------
        list of quartile arrays (one per XYZ channel)

        """
        if(pp.__coordinatesQuartiles_STORED is None):
            output = [[],[],[]];
            for i in range(3):
                output[i] = F.StatsGet123_Quartiles(pp.coordinates[:,i]);
            #end
            pp.__coordinatesQuartiles_STORED = output;
        #end
        return pp.__coordinatesQuartiles_STORED;
    #end
    
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
        if(isinstance(channel, str)):
            channel = F.ColorStringToInt(channel);
        #end
        if(not isinstance(channel, int)):
            raise Exception("Non-valid type for color channel");
        #end
        
        if(channel < 3):
            return pp.RGB[:,channel];
        else:
            return pp.HSV[:,channel-3];
        #end
    #end
    
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
        if(isinstance(channel, str)):
            channel = F.ColorStringToInt(channel);
        #end
        if(not isinstance(channel, int)):
            raise Exception("Non-valid type for color channel");
        #end
        
        if(channel < 3):
            return pp.RGB_Quartiles[channel];
        else:
            return pp.HSV_Quartiles[channel - 3];
        #end
    #end
    
    HSV = property(fget = __GetHSV)
    coordinate_Quartiles = property(fget = __GetCoordQuartiles)
    RGB_Quartiles = property(fget = __GetRGBQuartiles)
    HSV_Quartiles = property(fget = __GetHSVQuartiles)
    RGBHSV_Quartiles = property(fget = __GetRGBHSVQuartiles)
#end
