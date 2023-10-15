import numpy as np
import math as math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


class Object:
    def __init__(self, mass:float, position:np.ndarray, velocity:np.ndarray, crossSectionalArea:float):
        assert isinstance(position, np.ndarray), "Position argument must be a numpy array"
        assert position.shape == (3,), "Position argument must have 3 entries"
        assert isinstance(velocity, np.ndarray), "Velocity argument must be a numpy array"
        assert position.shape == (3,), "Velocity argument must have 3 entries"
        self.mass = mass
        self.position = position
        self.velocity = velocity
        self.crossSectionalArea = crossSectionalArea
        self.force = np.array([0,0,0])
        self.acceleration = np.array([0,0,0])

    def setForce(self, force:np.ndarray):
        self.force = force
        self.acceleration = force/self.mass
    
class Fluid:
    def __init__(self, density:float, dragCoefficient:float, windConfig:[dict]):
        self.density = density
        self.dragCoefficient = dragCoefficient
        self.windConfig = windConfig
        self.currentWindConfig = windConfig

    def findRelativeVelocity(self, absoluteVelocity:np.ndarray, time:float):
        if self.currentWindConfig == []:
            return absoluteVelocity
        if time<self.currentWindConfig[0]["duration"][0]:
            return absoluteVelocity
        if time>self.currentWindConfig[0]["duration"][1]:
            self.windConfig.pop(0)
            return self.findRelativeVelocity(absoluteVelocity, time)
        if time>=self.currentWindConfig[0]["duration"][0] and time<=self.currentWindConfig[0]["duration"][1]:
            return absoluteVelocity - self.windConfig[0]["velocity"]

    def fluidResitanceForce(self, crossSectionalArea:float, absoluteVelocity:np.ndarray, time:float):
        assert isinstance(absoluteVelocity, np.ndarray), "Velocity argument must be a numpy array"
        assert absoluteVelocity.shape == (3,), "Velocity argument must have 3 entries"
        relativeVelocity = self.findRelativeVelocity(absoluteVelocity, time)
        return -0.5*self.density*self.dragCoefficient*crossSectionalArea*np.linalg.norm(relativeVelocity)*relativeVelocity
    
class Simulator:
    def __init__(self, Object:Object, Fluid:Fluid, gravity:float=10, deltaTime:float = 1):
        self.Object = Object
        self.Fluid = Fluid
        self.gravity = gravity
        self.time = 0
        self.deltaTime = deltaTime
        self.history = {
            "positionHistory": [self.Object.position],
            "velocityHistory": [self.Object.velocity],
            "accelerationHistory": [self.Object.acceleration], 
            "forceHistory": [self.Object.force],
            "timeHistory": [0]
        }
    
    def forceComputer(self):
        # print(str(self.Fluid.fluidResitanceForce(self.Object.crossSectionalArea, self.Object.velocity)))
        currentForces = np.array([0,0,-self.gravity*self.Object.mass]) + self.Fluid.fluidResitanceForce(self.Object.crossSectionalArea, self.Object.velocity, self.time) 
        self.Object.setForce(currentForces)
    
    def compute(self):
        while True:
            self.time += self.deltaTime
            self.forceComputer()
            self.Object.position = self.Object.position + self.deltaTime*self.Object.velocity
            self.Object.velocity = self.Object.velocity + self.deltaTime*self.Object.acceleration
            self.history["positionHistory"].append(self.Object.position)
            self.history["velocityHistory"].append(self.Object.velocity)
            self.history["accelerationHistory"].append(self.Object.acceleration)
            self.history["forceHistory"].append(self.Object.force)
            self.history["timeHistory"].append(self.time)
            if self.Object.position[2] <= 0: break

def Plotter(positionHistory:[np.ndarray]):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    x = np.array([pos[0] for pos in positionHistory])
    y = np.array([pos[1] for pos in positionHistory])
    z = np.array([pos[2] for pos in positionHistory])
    ax.plot(x, y, z)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    plt.show()