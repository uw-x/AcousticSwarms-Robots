from enum import Enum
import numpy as np

## 1D ranging param
SOUND_SPEED = 344*100
DIS_BIAS = 1.8 #0.6
FS = 62500
# BACK_RANGE = 40
BACK_RANGE = 35
# BACK_RANGE = 30
# BACK_RANGE = 25


LOC_METHOD = "Simple"

## PF parameter
ALPHAS =  np.array([0.0025, 2.5e-05, 1.0, 2.5e-05])
BETAS = np.array([[0.49, 0.0, 0.0], [0.0, 0.49, 0.0], [0.0, 0.0, 0.49]])

USE_DEGREE = True
SAVE_DEBUG = True
PARTICLE_NUM = 100
SAVE_NAME = "EXP1"
VELOCITY_CLIP = 10
PRINT_DEBUG = False
SPEAKER_DIS = 0.3
RESET_APPLY = False
RESET_THRESHOLD = 8


## angle correction parameter
UPDATE_ANGLE = True
LEAST_POINT = 4

#map parameter
X_MAX = 150
Y_MAX = 150
Wall_X = -20
safe_guard = 6
platform_W = 12
platform_H = 18
ROBOT_NUM = 6
# ROBOT_NUM = 4

## Planner parameter
Astar_Epsilon = 0.75
ARRIVAL_Epsilon = 1.5
SETOFF_POINT = np.array([[-3.5], [20]])

## SMACOF para
SMACOF_MAX_ITER = 2000
SMACOF_CONVERGE = 1e-4

class Status(Enum):
    IDLE = 1
    FORWARD = 2
    ROTATE = 3



