
import numpy as np
from numpy import sin, cos
import time
import keyboard


from TrackDef import *
from CollisionChecks import *

#car
CAR_WIDTH = .5
CAR_LENGTH = 1.
RAD_VISION_LINES = 35.
NUM_VISION_LINES = 16

#movement prop
CAR_MAX_VEL = 1
CAR_ACC = 1
CAR_DRAG = .333
CAR_ROT_SPEED = 1.25



class Car:
    
    #rotation in rads from [1,0] counterclockwise
    def __init__(self):
        global lifeTime
        lifeTime = 0.0
        self.position = np.array([-15.,-15.])
        self.rotation = np.pi/2
        self.vel = np.array([0.,0.])
        self.visionLines = Car.genVisionLines()
        self.lastUpdate = time.time()
        self.nextUpdate = 0.0
        self.deltaT = 0.0
    
    def genVisionLines():
        radStep = 2*np.pi/NUM_VISION_LINES
        x = np.array([0.])
        y = np.array([0.])
        for i in range(NUM_VISION_LINES):
            x = np.concatenate((x, RAD_VISION_LINES*cos(i*radStep)), axis=None)
            y = np.concatenate((y, RAD_VISION_LINES*sin(i*radStep)), axis=None)
            x = np.concatenate((x, 0.), axis=None)
            y = np.concatenate((y, 0.), axis=None)
        return [x,y]

    def getKeyInput():
        inputs = [False,False,False,False]
        if keyboard.is_pressed('w'):
            inputs[0] = True
        if keyboard.is_pressed('s'):
            inputs[1] = True
        if keyboard.is_pressed('a'):
            inputs[2] = True
        if keyboard.is_pressed('d'):
            inputs[3] = True
        return np.array(inputs)
        
    def getRandomInput():
        return np.array([randBool(), randBool(), randBool(), randBool()])
        
    def update(self, inputs):
        self.nextUpdate = time.time()
        if(self.nextUpdate >= self.lastUpdate):
            self.deltaT = self.nextUpdate - self.lastUpdate
            
            self.__applyInputs(inputs)
            if self.__updatePhys():
                return True
            
            self.lastUpdate = self.nextUpdate
            return False
        
    def __applyInputs(self, inputs):
        if inputs[0]:
            self.vel += CAR_ACC * np.array([cos(self.rotation), sin(self.rotation)]) * self.deltaT
        if inputs[1]:
            self.vel -= CAR_ACC * np.array([cos(self.rotation), sin(self.rotation)]) * self.deltaT
        if inputs[2]:
            self.rotation += CAR_ROT_SPEED * self.deltaT
        if inputs[3]:
            self.rotation -= CAR_ROT_SPEED * self.deltaT
    
    def __updatePhys(self):
        self.__aplyDrag()
        if(self.__checkCollisions()):
            print("Hit a wall")
            print(self.position)
            self.__init__()
            return True
            
        if (np.linalg.norm(self.vel) > CAR_MAX_VEL):
            ang = np.arctan2(self.vel[1], self.vel[0])
            self.vel = CAR_MAX_VEL * np.array([cos(ang), sin(ang)])
        if(np.linalg.norm(self.vel) <= CAR_DRAG * self.deltaT):
            self.vel  = np.array([0.,0.])
        
        self.position += self.vel    
        return False
            
    def __aplyDrag(self):
        ang = np.arctan2(self.vel[1], self.vel[0])
        self.vel -= CAR_DRAG * np.array([cos(ang), sin(ang)]) * self.deltaT 

    def __checkCollisions(self):
        for i in range(len(OUTER_TRACK_PATH[0])-1):
            if(self.__checkIntersectingSeg([OUTER_TRACK_PATH[0][i:i+2],OUTER_TRACK_PATH[1][i:i+2]])):
                return True
        for i in range(len(INNER_TRACK_PATH[0])-1):
            if(self.__checkIntersectingSeg([INNER_TRACK_PATH[0][i:i+2],INNER_TRACK_PATH[1][i:i+2]])):
                return True    
        #print([OUTER_TRACK_PATH[0][:2],OUTER_TRACK_PATH[1][:2]])
        #return self.__checkIntersectingSeg([OUTER_TRACK_PATH[0][:2],OUTER_TRACK_PATH[1][:2]])
        return False
        
    def __checkIntersectingSeg(self,ab):
        car = self.getPlot()
        
        #print("start")
        #print(ab)
        for i in range(4):
            #print("i: "+ str(i))
            #print([car[0][i:i+2],car[1][i:i+2]])
            if(isIntersecting([car[0][i:i+2],car[1][i:i+2]],ab)):
                return True
        #print("done")
        return False
    
    def getPlot(self):
        xPts = CAR_WIDTH * np.array([1./2, 1./2, -1./2, -1./2, 1./2, 0., -1./2])
        yPts = CAR_LENGTH * np.array([-1./2, 1./2, 1./2, -1./2, -1./2, 0., -1./2])
        return self.carFrameTransform([xPts, yPts])
    
    def getVisionLines(self):
        return self.carFrameTransform(self.visionLines)   
    
    def getVisionCollisions(self):
        visionLines = self.getVisionLines()
        trackLines = np.concatenate((INNER_LINES, OUTER_LINES))
        visionIntersects = []
        
        for i in range(0,NUM_VISION_LINES*2, 2):
            tmp = getClosestIntersect([visionLines[0][i:i+2],visionLines[1][i:i+2]], trackLines)
            if(tmp != False):
                visionIntersects.append(tmp)
        visionIntersects = np.array(visionIntersects).T
        if(len(visionIntersects) == 2):
            return visionIntersects
        return np.array([[],[]])
    
    def carFrameTransform(self, data):
        c = cos(self.rotation-np.pi/2)
        s = sin(self.rotation-np.pi/2)
        rotated =  np.dot([[c,-s],[s,c]], data)
        return np.array([self.position[0] + rotated[0], \
        self.position[1] + rotated[1]])
    
        