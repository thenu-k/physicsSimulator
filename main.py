from System.system import Object, Fluid, Simulator, Plotter
import numpy as np
import matplotlib.pyplot as plt

object = Object(
    mass = 1,
    position=np.array([0,0,100]),
    velocity=np.array([40, 0, 0]),
    crossSectionalArea=0.05
)

fluid = Fluid(
    density=1,
    dragCoefficient=1,
    windConfig = [
        {
            "duration": [2,3],
            "velocity": np.array([0,30,0])
        },
        {
            "duration": [3,4],
            "velocity": np.array([0,0,50])
        }
    ]
)

simulator = Simulator(
    Object=object,
    Fluid=fluid,
    gravity=10,
    deltaTime=0.01
)

simulator.compute()
positionHistory = simulator.history["positionHistory"]
forceHistory = simulator.history["forceHistory"]
velocityHistory = simulator.history["velocityHistory"]
Plotter(positionHistory=positionHistory)