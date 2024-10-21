#!/usr/bin/env python
from DroneInterface import *
from DroneProperties import *
import numpy as np
from Controller import Controller
from scipy.io import loadmat

###############################################################################
###############################################################################
def main():

    ### Define trajectories
    pd,vd,ad = {},{},{}
    td = np.r_[0:30:0.1]   # Vector of time for trajectory.
    
    ID = 1
    # # Eight like trajectory
    # T = 15
    # w = 2*np.pi/T
    # r = 1
    # pd[ID] = np.block([[np.sin(2*(w*td-np.pi/2.0))],[2*np.cos(w*td-np.pi/2.0)],[1.2+0.4*np.cos(2*w*td-np.pi)]])
    # vd[ID] = np.block([[2*w*np.cos(2*(w*td-np.pi/2.0))],[-2*w*np.sin(w*td-np.pi/2.0)],[-w*0.8*np.sin(2*w*td-np.pi)]])
    # ad[ID] = np.block([[-4*w**2*np.sin(2*(w*td-np.pi/2.0))],[-2*w**2*np.cos(w*td-np.pi/2.0)],[-1.6*w**2*np.cos(2*w*td-np.pi)]])
    
    # # Waypoint
    # pd[ID] = np.block([[-1*np.ones_like(td)],[0*np.ones_like(td)],[1*np.ones_like(td)]])
    # vd[ID] = np.block([[0*np.ones_like(td)],[0*np.ones_like(td)],[0*np.ones_like(td)]])
    # ad[ID] = np.block([[0*np.ones_like(td)],[0*np.ones_like(td)],[0*np.ones_like(td)]])
    # yawd = 0*td + np.pi/2
    
    # Circle
    T = 30
    w = 2*np.pi/T
    r = 2.0
    pd[ID] = np.block([[r*(np.cos(w*td)-1)],[r*np.sin(w*td)],[2*np.ones_like(td)]])
    vd[ID] = np.block([[-r*w*np.sin(w*td)],[r*w*np.cos(w*td)],[0*np.ones_like(td)]])
    ad[ID] = np.block([[-r*w**2*np.cos(w*td)],[-r*w**2*np.sin(w*td)],[0*np.ones_like(td)]])
    yawd = w*td + np.pi/2

    ############################################################################
    ############    Create drones
    ############################################################################
    drone_list = []

    ###     SITL Drones
    IDs_sitl = [ID]
    controllers_sitl = {}
    drones_sitl = []
    mode = "sitl"
    for ID in IDs_sitl:
        controllers_sitl[ID] = Controller(ID,Iris)
        controllers_sitl[ID].set_desired_trajectory(td,pd[ID],vd[ID],ad[ID],yawd)
        drones_sitl.append(Drone(ID,mode,controllers_sitl[ID].control_law))
    drone_list = drone_list + drones_sitl

    ###     Real Drones
    IDs_real = []
    controllers_real = {}
    drones_real = []
    mode = "real"
    for ID in IDs_real:
        if ID == 7:
            controllers_real[ID] = Controller(ID,Kopis)
        else:
            controllers_real[ID] = Controller(ID,IntelAero)
        #controllers_real[ID].set_2D_desired_trajectory(td,pd[:,:,ID-5],vd[:,:,ID-5],ad[:,:,ID-5],yawd,1)
        controllers_real[ID].set_desired_trajectory(td,pd[ID],vd[ID],ad[ID],yawd)
        drones_real.append(Drone(ID,mode,controllers_real[ID].control_law))
    drone_list = drone_list + drones_real

    ##################################################################
    ##  Manager initialization
    ##################################################################
    manager = DroneManager(drone_list,takeoff_alt_=1)



###############################################################################
###############################################################################
if __name__=='__main__':
    main()
