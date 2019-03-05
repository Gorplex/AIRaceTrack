

from CarController import *

#wraps Car class to run game and collect data

class DataCollector:

    def __init__(self):
        self.data = []
        self.car = Car()
        #self.lastRadar = None
    def update(self, input):
        
        pos = self.car.getPos()
        vel = self.car.getVel()
        radar = self.car.getRadar()
        score = 0.0
        
        retVal = self.car.update(input)
        radarNext = self.car.getRadar()
        radarDot = []
        for i in range(len(radar)):
            xDot = radarNext[i][0] - radar[i][0]
            yDot = radarNext[i][1] - radar[i][1]
            radarDot = np.append(radarDot, [xDot,yDot], axis=0)
        observations = np.array([radar,radarDot])
        
        dataPoint = DataPoint(pos, vel, observations, input, score)
        self.data  = np.append(self.data, dataPoint)
        return retVal
        
    def getData(self):
        return self.data
    def getCar(self):
        return self.car
        
class DataPoint:
    def __init__(self, pos, vel, observations, input, score):
        self.pos = pos
        self.vel = vel
        self.observations = observations
        self.input = input
        self.score = score
    def __repr__(self):
        return str(self.pos)