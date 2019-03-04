
def __intesectingHelper(ab1, ab2):
    x1 = ab1[0][0]
    x2 = ab1[0][1]
    x3 = ab2[0][0]
    x4 = ab2[0][1]
    y1 = ab1[1][0]
    y2 = ab1[1][1]
    y3 = ab2[1][0]
    y4 = ab2[1][1]    
    #collision Dection from http://www.jeffreythompson.org/collision-detection/line-line.php
    uA = ((x4-x3)*(y1-y3) - (y4-y3)*(x1-x3)) / ((y4-y3)*(x2-x1) - (x4-x3)*(y2-y1));
    uB = ((x2-x1)*(y1-y3) - (y2-y1)*(x1-x3)) / ((y4-y3)*(x2-x1) - (x4-x3)*(y2-y1));
    return [uA, uB]

def isIntersecting(ab1, ab2):
    uA,uB = __intesectingHelper(ab1, ab2)
    if (uA >= 0 and uA <= 1 and uB >= 0 and uB <= 1):
        return True;
    return False;

def getIntersect(ab1, ab2):
    x1 = ab1[0][0]
    x2 = ab1[0][1]
    y1 = ab1[1][0]
    y2 = ab1[1][1]
    uA,uB = __intesectingHelper(ab1, ab2)
    if (uA >= 0 and uA <= 1 and uB >= 0 and uB <= 1):
        interX = x1 + (uA * (x2-x1));
        interY = y1 + (uA * (y2-y1));
        return [interX, interY]
    return False

def getClosestIntersect(ab1, ab2list):
    x1 = ab1[0][0]
    x2 = ab1[0][1]
    y1 = ab1[1][0]
    y2 = ab1[1][1]
    bestX = float("inf")
    bestY = float("inf")
    for ab2 in ab2list:
        uA,uB = __intesectingHelper(ab1, ab2)
        if (uA >= 0 and uA <= 1 and uB >= 0 and uB <= 1):
            interX = x1 + (uA * (x2-x1));
            interY = y1 + (uA * (y2-y1));
            if(abs(interX-x1)<abs(bestX-x1)):
                bestX=interX
                bestY=interY
    if(bestX < float("inf")):
        return [bestX, bestY]
    return False
