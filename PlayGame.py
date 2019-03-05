
import time
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import random


from CarController import *
from TrackDef import *



RANDOM_CONTOLL = False


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
plt.xticks([])
plt.yticks([])
plt.grid(False)

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
    
    if RANDOM_CONTOLL:
        car.update(Car.getRandomInput())
    else:
        car.update(Car.getKeyInput())
  
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