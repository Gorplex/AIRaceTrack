
import numpy as np
from numpy import sin, cos
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time

import keyboard

from CollisionChecks import *

#grid size
CORD_X = 35
CORD_Y = 20
INNER_TRACK_PATH = np.array([[-14.,-14.,14.,-10.,-10.,14.,-14.], [-14.,14.,14.,6.,-6.,-14.,-14.]])
OUTER_TRACK_PATH = np.array([[-17.,-17.,17.,17.,-5.,-5.,17.,17.,-17.], [-17.,17.,17.,10.,4.,-4.,-10.,-17.,-17.]])
INNER_LINES = []
OUTER_LINES = []


def PointsToLineSegments(points):
    lineList = []  
    for i in range(len(points[0])-1):
        lineList.append(np.array([points[0][i:i+2],points[1][i:i+2]]))
    return np.array(lineList)

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

def checkKeys():
    inputs = [False,False,False,False]
    if keyboard.is_pressed('w'):
        inputs[0] = True
    if keyboard.is_pressed('s'):
        inputs[1] = True
    if keyboard.is_pressed('a'):
        inputs[2] = True
    if keyboard.is_pressed('d'):
        inputs[3] = True
    return inputs

class Car:
    
    #rotation in rads from [1,0] counterclockwise
    def __init__(self):
        global lifeTime
        lifeTime = 0.0
        self.position = np.array([-15.,-15.])
        self.rotation = np.pi/2
        self.vel = np.array([0.,0.])
        self.visionLines = genVisionLines()
        self.lastUpdate = time.time()
        self.nextUpdate = 0.0
        self.deltaT = 0.0
        
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
    
        

#------------------------------------------------------------
# set up initial state and global variables
dt = 1./40 # 40 fps
realTime  = 0.0
satrtRealTime = time.time()
lastRealTime = time.time()
frameTime = 0.0
frameCount = 0
frameRate = 0.0
lifeTime = 0.0
car = Car()


#------------------------------------------------------------
# set up figure and animation
fig = plt.figure()
ax = fig.add_subplot(111, aspect='equal', autoscale_on=False,
                     xlim=(-CORD_X, CORD_X), ylim=(-CORD_Y, CORD_Y))
#ax.grid()

#car lines
line, = ax.plot([], [], lw=1)
visionPlot, = ax.plot([],[], lw=.25)
visionDots, = ax.plot([],[], linestyle='None', marker='o')

real_time_text = ax.text(0.02, 0.95, '', transform=ax.transAxes)
frame_time_text = ax.text(0.02, 0.90, '', transform=ax.transAxes)
frame_rate_text = ax.text(0.02, 0.85, '', transform=ax.transAxes)
life_time_text = ax.text(0.02, 0.80, '', transform=ax.transAxes)

#track lines
inner_track, = ax.plot(INNER_TRACK_PATH[0], INNER_TRACK_PATH[1], lw=.5)
outer_track, = ax.plot(OUTER_TRACK_PATH[0], OUTER_TRACK_PATH[1] , lw=.5)

def initAnimation():
    """initialize animation"""
    line.set_data([], [])
    visionPlot.set_data([], [])
    visionDots.set_data([], [])
    real_time_text.set_text('')
    frame_time_text.set_text('')
    frame_rate_text.set_text('')
    life_time_text.set_text('')
    return line, visionPlot, visionDots, real_time_text, frame_time_text, frame_rate_text, life_time_text
    
def animate(i):
    """perform animation step"""
    global  dt, realTime, lastRealTime, satrtRealTime, frameTime, frameCount, frameRate, lifeTime
    
    frameTime += dt
    frameCount += 1
    nextRealTime = time.time()
    if(lastRealTime !=  nextRealTime):
        realTime = nextRealTime - satrtRealTime
        frameRate = frameCount/(nextRealTime - lastRealTime)
        frameCount = 0
        lastRealTime = nextRealTime
    
    car.update(checkKeys())
  
    line.set_data(car.getPlot())
    visionPlot.set_data(car.getVisionLines())
    visionDots.set_data(car.getVisionCollisions())
    real_time_text.set_text('Real Time %.1f' % realTime)
    frame_time_text.set_text('Frame Time %.1f' % frameTime)
    frame_rate_text.set_text('Frame Rate %.1f' % frameRate)
    life_time_text.set_text('Life Time %.1f' % lifeTime)
    return line, visionPlot, visionDots, real_time_text, frame_time_text, frame_rate_text, life_time_text


def main():
    global INNER_LINES, OUTER_LINES
    #disable saving on the s key
    plt.rcParams['keymap.save'] = ''
    
    INNER_LINES = PointsToLineSegments(INNER_TRACK_PATH)
    OUTER_LINES = PointsToLineSegments(OUTER_TRACK_PATH)
    
    # choose the interval based on dt and the time to animate one step
    t0 = time.time()
    animate(0)
    animate(0)
    t1 = time.time()
    interval = (1000 * dt - (t1 - t0))/2
    
    ani = animation.FuncAnimation(fig, animate, frames=300,
                              interval=interval, blit=True, init_func=initAnimation)
    
    plt.tight_layout(pad=0, w_pad=0, h_pad=0)
    plt.show()




if __name__ == "__main__":
    main()