import numpy as np

_IntelAero_IDs = [1,2,3,4]
_SnapDragon_IDs = [5,6]
_Kopis_IDs = [7,8,9,10]

def properties_from_ID(ID,mode):
    if mode == 'real':
        if ID in _IntelAero_IDs:
            return IntelAero
        if ID in _SnapDragon_IDs:
            return Snap
        if ID in _Kopis_IDs:
            return Kopis
    else:
        return Iris

class IntelAero:
    ### Thrust curve
    _a = 8.84
    _b = 2.955
    _c = -1.478
    _d = 10.37

    ### Physical properties
    mass = 1.38      # Mass in kg
    radius = 0.4    # Radius of circle involving the drone (in meters)

    ### Saturation properties
    max_accel = 4
    max_vel = 2

    ### Gains for trajectory tracking
    Kp = 4
    Kv = 4
    Ki = 0

    ### Transform thrust in normalized input using the thrust curve
    @staticmethod
    def InputFromThrust(Thrust,v=0):
        if Thrust < 0:
            return 0
        Input = (np.tan((Thrust-IntelAero._d)/IntelAero._a)-IntelAero._c)/IntelAero._b
        if Input < 0:
            return 0
        if Input > 1:
            return 1
        return Input

class Snap:
    ### Thrust curve
    _a = 0.1377
    _b = 0.003759
    _c = -0.02976
    _d = -0.05973

    ### Physical properties
    mass = 0.56      # Mass in kg
    radius = 0.4    # Radius of circle involving the drone (in meters)

    ### Saturation properties
    max_accel = 17
    max_vel = 2

    ### Gains for trajectory tracking
    Kp = 4
    Kv = 4
    Ki = 0.4

    ### Transform thrust in normalized input using the thrust curve
    @staticmethod
    def InputFromThrust(Thrust,v=0):
        if Thrust < 0:
            return 0
        Input = 0.23/0.345*(Snap._a*np.exp(-Snap._c*Thrust)*np.sqrt(Thrust) + Snap._b*Thrust + Snap._d)
        #Input = (np.tan((Thrust-IntelAero._d)/IntelAero._a)-IntelAero._c)/IntelAero._b
        if Input < 0:
            return 0
        if Input > 1:
            return 1
        return Input

class Kopis:
    ### Thrust curve
    _a = 0.0001531
    _b = -0.002836
    _c = 0.07347
    _d = -0.003982

    ### Physical properties
    mass = 0.39      # Mass in kg
    radius = 0.4    # Radius of circle involving the drone (in meters)

    ### Saturation properties
    max_accel = 4

    ### Gains for trajectory tracking
    Kp = 4
    Kv = 4
    Ki = 0.2

    ### Transform thrust in normalized input using the thrust curve
    @staticmethod
    def InputFromThrust(Thrust,v=0):
        if Thrust < 0:
            return 0
        Input = Kopis._a*Thrust**3 + Kopis._b*Thrust**2 + Kopis._c*Thrust + Kopis._d
        if Input < 0:
            return 0
        if Input > 1:
            return 1
        return Input


class Iris:

    ### Thrust curve
    _a = 34.068
    _b = 7.1202
    _b2 = _b**2
    _c = 0.0074158
    _i0 = 0.561
    _G = 9.80665

    ### Physical properties
    mass = 1.52      # Mass in kg
    radius = 0.45    # Radius of circle involving the drone (in meters)

    ### Saturation properties
    max_accel = 4
    max_vel = 1

    ### Gains for trajectory tracking
    Kp = 4
    Kv = 4
    Ki = 0.2

    ### Transform thrust in normalized input using the thrust curve
    @staticmethod
    def InputFromThrust(Thrust,v):
        if Thrust < 0:
            return 0
        Input = (-Iris._b+np.sqrt(Iris._b2-4*Iris._a*(Iris._c-Thrust/(1-v/25))))/(2*Iris._a)
        #Input = (Thrust/(1-v/25) - Iris.mass*Iris._G)/(2*Iris._a*Iris._i0 + Iris._b) + Iris._i0
        if Input < 0:
            return 0
        if Input > 1:
            return 1
        return Input
