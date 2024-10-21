###############################################################################
### Module imports
###############################################################################

### To perform matrix and matematical computations
import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.interpolate import interp1d
from time import time                           ### To do some time stuff
from DroneInterface import Sampling_Time as h   ### To obtain the implemented sampling times
from DroneProperties import Iris                ### Import properties to set a default
from scipy.io import loadmat
#import UDP_Connections



###############################################################################
###     Class: Controller
### This class implements the projected controller
###############################################################################
class Controller:

    IDS = []
    DRONES = []
    G = 9.80665
    e_3 = np.array((0,0,1))
    Gxe_3 = G*e_3

    ###########################################################################
    ### __init__() method - Class constructor
    ###########################################################################
    def __init__(self, ID, properties=Iris):
        self.properties = properties
        self.ID = ID
        self.__initialize_variables__()
        #self.rcv_setpnt = UDP_Connections.CommunicateWithPlanner(self.ID)
        Controller.IDS.append(ID)
        Controller.DRONES.append(self)

    ###########################################################################
    ### __initialize_variables__() method - fills some values when creating
    ### an instance of the class
    ###########################################################################
    def __initialize_variables__(self):
        self.p = np.zeros(3)
        self.v = np.zeros(3)
        self.t = 0
        self.alt = 1
        self.desired_mode = "2D"
        self.integrator = np.zeros(3)
        self.u = np.zeros(3)
        td = np.array((0.,1000.))
        self.pd = interp1d(td, np.zeros((2,len(td))), axis=-1,
            bounds_error=False, fill_value=(np.zeros(2),np.zeros(2)),assume_sorted=True)
        self.vd = interp1d(td, np.zeros((2,len(td))), axis=-1,
            bounds_error=False, fill_value=(np.zeros(2),np.zeros(2)),assume_sorted=True)
        self.ad = interp1d(td, np.zeros((2,len(td))), axis=-1,
            bounds_error=False, fill_value=(np.zeros(2),np.zeros(2)),assume_sorted=True)


    ###########################################################################
    ### __input_transformation__() method - This method receives the desired
    ### yaw and acceleration vector, and transforms it into a Thrust value and
    ### and attitude
    ###########################################################################
    def __input_transformation__(self,accel,yaw_d):
        # Saturate accelerations
        accel[accel>self.properties.max_accel] = self.properties.max_accel
        accel[accel<-self.properties.max_accel] = -self.properties.max_accel
        # Compute transformation
        #print(self.r.as_euler('xyz')[2])
        Rz = R.from_euler('xyz',(0,0,self.r.as_euler('xyz')[2]))
        Thrust = self.properties.mass*Rz.inv().apply(accel+Controller.Gxe_3)
        pitch_d = np.arctan2(Thrust[0],Thrust[2])
        roll_d = np.arctan2(-Thrust[1],np.linalg.norm(Thrust[[0,2]]))
        Thrust = np.linalg.norm(Thrust)
        v = np.linalg.norm(self.v)
        Input = self.properties.InputFromThrust(Thrust,v)
        Attitude = R.from_euler('xyz',(roll_d,pitch_d,yaw_d))
        return Input, Attitude


    ###########################################################################
    ### set_2D_desired_trajectory() method - describe a desired 2D trajectory,
    ### in time. Receives the desired position, velocity and acceleration,
    ### as well as the corresponding time. These must be described using
    ### a numpy array, shaped (2,N), where N represents the number of entries
    ### in the time vector
    ###########################################################################
    def set_2D_desired_trajectory(self,td,pd,vd,ad,yawd,alt=1):
        self.pd = interp1d(td, pd, axis=-1, bounds_error=False,
            fill_value=(pd[:,0],pd[:,-1]),assume_sorted=True)
        self.vd = interp1d(td, vd, axis=-1, bounds_error=False,
            fill_value=(np.zeros(2),np.zeros(2)),assume_sorted=True)
        self.ad = interp1d(td, ad, axis=-1, bounds_error=False,
            fill_value=(np.zeros(2),np.zeros(2)),assume_sorted=True)
        self.yawd = interp1d(td, yawd, axis=-1, bounds_error=False,
            fill_value=(yawd[0],yawd[-1]),assume_sorted=True)
        self.alt = alt
        self.desired_mode = "2D"


    ###########################################################################
    ### set_desired_trajectory() method - receives a string with a file from
    ### MatLab (.mat) to define the desired trajectory
    ###########################################################################
    def set_desired_trajectory(self,td,pd,vd,ad,yawd):
        self.pd = interp1d(td, pd, axis=-1, bounds_error=False,
            fill_value=(pd[:,0],pd[:,-1]),assume_sorted=True)
        self.vd = interp1d(td, vd, axis=-1, bounds_error=False,
            fill_value=(np.zeros(3),np.zeros(3)),assume_sorted=True)
        self.ad = interp1d(td, ad, axis=-1, bounds_error=False,
            fill_value=(np.zeros(3),np.zeros(3)),assume_sorted=True)
        self.yawd = interp1d(td, yawd, axis=-1, bounds_error=False,
            fill_value=(yawd[0],yawd[-1]),assume_sorted=True)
        self.desired_mode = "3D"


    ###########################################################################
    ### control_law() method . This computes the actuation to be applied to
    ### the Drone
    ###########################################################################
    def control_law(self,t,p,v,rotation):
        # Reset integrator if it was not running for a while
        if np.abs(t - self.t) > 0.5:
            self.integrator = np.zeros(3)

        # Update stored data
        self.t, self.p, self.v = t, p, v
        self.r, euler = rotation, rotation.as_euler('xyz')

        # Get desired trajectory
        if self.desired_mode == "2D":
            pd = np.block([self.pd(t),self.alt])
            vd = np.block([self.vd(t),0])
            ad = np.block([self.ad(t),0])
            yawd = self.yawd(t)
        if self.desired_mode == "3D":
            pd = self.pd(t)
            vd = self.vd(t)
            ad = self.ad(t)
            yawd = self.yawd(t)
        #if self.desired_mode is "receive":
        #    pd, vd, ad = self.rcv_setpnt.get_setpoint()
        #    yawd = 0

        # Update integral term
        idot = (p-pd)
        self.integrator += idot*h
        self.integrator[self.integrator>5] = 5
        self.integrator[self.integrator<-5] = -5

        # Compute actuation
        u_fb = -self.properties.Kv*(v-vd) - self.properties.Kp*(p-pd) - self.properties.Ki*self.integrator
        u = u_fb + ad
        T, att = self.__input_transformation__(u,yawd)
        #print(T)

        return T, att, u
