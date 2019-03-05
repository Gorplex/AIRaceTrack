

import numpy as np

#grid size
CORD_X = 35
CORD_Y = 20
INNER_TRACK_PATH = np.array([[-14.,-14.,14.,-10.,-10.,14.,-14.], [-14.,14.,14.,6.,-6.,-14.,-14.]])
OUTER_TRACK_PATH = np.array([[-17.,-17.,17.,17.,-5.,-5.,17.,17.,-17.], [-17.,17.,17.,10.,4.,-4.,-10.,-17.,-17.]])


def PointsToLineSegments(points):
    lineList = []  
    for i in range(len(points[0])-1):
        lineList.append(np.array([points[0][i:i+2],points[1][i:i+2]]))
    return np.array(lineList)

INNER_LINES = PointsToLineSegments(INNER_TRACK_PATH)
OUTER_LINES = PointsToLineSegments(OUTER_TRACK_PATH)
    
