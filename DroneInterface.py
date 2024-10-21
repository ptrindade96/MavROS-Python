###############################################################################
### Module imports
###############################################################################

### For creating MavROS processes
from subprocess import Popen, PIPE
import signal
import os

### Messages for use with MavROS
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import ExtendedState
from mavros_msgs.msg import State
from mavros_msgs.msg import AttitudeTarget
from mavros_msgs.msg import PositionTarget
from mavros_msgs.srv import *

### Interface with ROS
import rospy

### To create a GUI and manage program
import tkinter as tk
import sys

### To perform matrix and matematical computations
import numpy as np
from scipy.spatial.transform import Rotation as R

### To implement threads of execution
from threading import Thread

### To handle time
from time import sleep
from time import time

### To log data to a .mat file
from scipy.io import savemat

### To determine type of drone
from DroneProperties import Snap, Kopis, properties_from_ID


###############################################################################
###     Definitions - Some global variables
###############################################################################
Sampling_Frequency = 30.
Sampling_Time = 1/Sampling_Frequency
Logging_Rate = 100.

###############################################################################
###     Class: Drone
### This class provides a higher abstraction to the interface with the Drone.
### It represents a Drone and its properties.
###############################################################################
class Drone:

    IDs = []

    ###########################################################################
    ### __init__() method - Class constructor
    ###########################################################################
    def __init__(self, ID=1, mode="sitl", controller=None, log=True):
        error = "Connection with given ID already exists"
        assert (ID not in Drone.IDs), error
        self.ID = ID
        self.mode = mode
        self.controller = controller
        self.__initialize_variables__()
        self.Thread = Thread(target=self.__threadCode__)
        self.Thread.setName("Drone_"+str(self.ID)+" thread")
        self.Thread.setDaemon(True)
        self.log = log
        if self.log:
            self.logger = Logger(self.ID)
        Drone.IDs.append(ID)

    ###########################################################################
    ### connect() method - starts drone communications and controller thread
    ###########################################################################
    def connect(self):
        self.connection = Connection(self.ID,self.mode)
        self.rate = rospy.Rate(Sampling_Frequency)
        self.run_thread = True
        self.Thread.start()

    ###########################################################################
    ### land() method - send a land command
    ###########################################################################
    def land(self):
        self.connection.set_mode('AUTO.LAND')
        self.in_hover = False
        self.controlling = False

    ###########################################################################
    ### switch_to_controller() method - switch to controller
    ###########################################################################
    def switch_to_controller(self):
        if self.controlling:
            print("Already in controller.")
            return
        if self.controller is None:
            print("Feature not available. No controller provided")
            return
        try:
            self.controller(0, self.ID*np.ones(3), np.zeros(3), R.from_euler('xyz',np.zeros(3)))
        except:
            print("Provided controller has thrown error. Not switching.")
            return
        if self.connection.mode != 'OFFBOARD':
            self.connection.set_mode('OFFBOARD')
        self.T0 = rospy.get_time()
        self.controlling = True
        self.in_hover = False


    ###########################################################################
    ### hover() method - switch to position hold. Stays at current position
    ### with given altitude
    ###########################################################################
    def hover(self, alt=None, yaw=None):
        if self.connection.mode != 'OFFBOARD':
            self.connection.set_mode('OFFBOARD')
        self.hover_position = self.p
        self.hover_yaw = self.r.as_euler('xyz')[2]
        if yaw != None:
            self.hover_yaw = yaw
        if alt != None:
            self.hover_position[2] = alt
        self.in_hover = True
        self.controlling = False

    ###########################################################################
    ### arm() method - send an arm command to the drone
    ###########################################################################
    def arm(self):
        if not self.connection.is_["landed"]:
            print("Drone "+str(self.ID)+"is flying. Not sending arm command.")
            return
        if self.connection.is_['armed']:
            return
        self.connection.set_mode('OFFBOARD')
        self.connection.send_arm_command()

    ###########################################################################
    ### disarm() method - send a protected disarm command to the drone
    ###########################################################################
    def disarm(self):
        if not self.connection.is_["landed"]:
            print("Drone "+str(self.ID)+" is not landed. Land first.")
            return
        self.connection.send_disarm_command(force=False)

    ###########################################################################
    ### emergency_disarm() method - send a disarm command to the drone. Does
    ### not check if it is landed.
    ###########################################################################
    def emergency_disarm(self):
        self.connection.send_disarm_command(force=True)

    ###########################################################################
    ### disconnect() method
    ###########################################################################
    def disconnect(self):
        self.run_thread = False
        self.Thread.join(timeout=1)
        if self.log:
            try:
                self.logger.save_data()
            except Exception as e:
                print("Thrown an error saving the log: " + str(e))
        if self.connection:
            self.connection.disconnect()
            self.connection = []
        if self.Thread.isAlive:
            return False
        return True

    ###########################################################################
    ### __threadCode__() method - This is the thread running code
    ###########################################################################
    def __threadCode__(self):
        while self.run_thread:
            if self.connection.is_["connected"]:
                self.p,self.v,self.r = self.connection.get_data()
                self.p_ = self.p + self.origin
                if not self.in_hover and not self.controlling:
                    self.connection.send_att_thrust_command(0,R.from_euler('xyz',np.zeros(3)))
                    self.rate.sleep()
                elif self.in_hover:
                    self.connection.send_position_command(self.hover_position,self.hover_yaw)
                    self.rate.sleep()
                else:
                    t = rospy.get_time() - self.T0
                    T,att,self.u = self.controller(t, self.p_, self.v, self.r)
                    self.rate.sleep()
                    self.connection.send_att_thrust_command(T,att)

    ###########################################################################
    ### __initialize_variables__() method - fills some values when creating
    ### an instance of the class
    ###########################################################################
    def __initialize_variables__(self):
        self.p = np.array((0,0,0))
        self.v = np.array((0,0,0))
        self.u = np.array((0,0,0))
        self.r = R.from_euler('xyx',(0,0,0))
        self.controlling = False
        self.in_hover = False
        self.origin = np.array((0.0,0.0,0.0))
        if self.mode.startswith("sitl"):
            self.origin[0] = 3*(self.ID-1)
        self.p_ = self.origin
        self.T0 = 0

    ###########################################################################
    ### __del__() method - Class destructor
    ###########################################################################
    def __del__(self):
        if self.run_thread:
            self.run_thread = False
            self.Thread.join(timeout=1)
        if self.connection:
            self.connection.disconnect()
            self.connection = []
        Drone.IDs.remove(self.ID)


###############################################################################
###     Class: Connection
### This class implements the connection to a Drone. It can be use to interact
### with the Drone, by sending commands and receiving data.
###############################################################################
class Connection:

    droneIDs = []

    ###########################################################################
    ### __init__() method - Class constructor
    ###########################################################################
    def __init__(self, droneID=1, mode="sitl"):
        error = "Connection with given ID already exists"
        assert (droneID not in Connection.droneIDs), error
        self.__start_mavros__(droneID,mode)
        if properties_from_ID(droneID,mode) == Snap or properties_from_ID(droneID,mode) == Kopis:
            sleep(2)
            self.__start_send_pos__(droneID)
        if not Connection.droneIDs:
            sleep(3)
            rospy.init_node("DroneController",anonymous=False)
        self.__initialize_variables__(droneID)
        self.__start_subscribers__()
        self.__create_publishers__()
        Connection.droneIDs.append(self.droneID)

    ###########################################################################
    ### __initialize_variables__() method - To be called in the constructor to
    ### create and initialize some of the instance variables
    ###########################################################################
    def __initialize_variables__(self,id):
        self.droneID = id
        self.p = np.zeros(3)
        self.v = np.zeros(3)
        self.r = R.from_euler('xyz',(0,0,0))
        self.is_ = {"armed":False, "landed":True, "connected":False}
        self.mode = "AUTO.LAND"

    ###########################################################################
    ### __start_mavros__() method - To be called in the constructor to start
    ### the MavROS node associated with this vehicle
    ###########################################################################
    def __start_mavros__(self,droneID,mode="sitl"):
        # Compute appropriate command line to spawn MavROS
        namespace = "Drone_"+str(droneID)
        cmd = ["roslaunch","mavros","px4.launch"]
        if mode == "sitl":
            id = (droneID-1)
            cmd.append('fcu_url:=udp://localhost:'+str(14540+id)+'@localhost:'+str(14580+id))
            #cmd.append('fcu_url:=udp://localhost:'+str(14030+id)+'@localhost:'+str(14280+id)) # For smaller bandwidth
            cmd.append('tgt_system:='+str(droneID))
        elif mode == "sitl_remote":
            p = 25000 + droneID
            p_ = 25000 - droneID
            cmd.append('fcu_url:=udp://:'+str(p)+'@10.42.0.2:'+str(p_))
            p = 14540 + 10*droneID
            cmd.append('gcs_url:=udp://:'+str(p-1)+'@localhost:'+str(14550))
            cmd.append('tgt_system:='+str(droneID))
            cmd.append('mavros_sys_id:=100')
            cmd.append('mavros_comp_id:=240')
            cmd.append('tgt_component:=1')
        elif mode == "real":
            p = 15000 + droneID
            cmd.append('fcu_url:=udp://:'+str(p)+'@')
            cmd.append('tgt_system:='+str(droneID))
            cmd.append('mavros_sys_id:=100')
            cmd.append('mavros_comp_id:=240')
            cmd.append('tgt_component:=1')
            cmd.append('gcs_url:=udp://@localhost:'+str(14550))
        # Launch MavRos process associated to Drone, without output
        devnull = open('/dev/null', 'w')
        print(cmd)
        self.mavROS = Popen(cmd, stderr=devnull, stdout=devnull,
            env=dict(os.environ, ROS_NAMESPACE=namespace))

    ###########################################################################
    ### __start_send_pos__() method - To be called in the constructor to start
    ### the sending mocap data to the drone
    ###########################################################################
    def __start_send_pos__(self,droneID):
        namespace = "Drone_"+str(droneID)
        cmd = ["roslaunch","vrpn_client_ros","arena.launch"]
        devnull = open('/dev/null', 'w')
        print(cmd)
        self.vrpn = Popen(cmd, stdout=devnull, stderr=devnull ,
            env=dict(os.environ, ROS_NAMESPACE=namespace))
        cmd = ["rosrun","topic_tools","drop","/"+namespace+"/vrpn_client_node/drone"+str(droneID)+"/pose","5","6","/"+namespace+"/mavros/vision_pose/pose"]
        print(cmd)
        self.remap = Popen(cmd, stdout=devnull, stderr=devnull,
            env=dict(os.environ, ROS_NAMESPACE=namespace))

    ###########################################################################
    ### __start_subscribers__() method - To be called in the constructor to
    ### initialize the subscribers to the MavROS messages
    ###########################################################################
    def __start_subscribers__(self):
        self.Subscribers = []
        namespace = "/Drone_"+str(self.droneID)
        topic = namespace+"/mavros/local_position/pose"
        self.Subscribers.append(rospy.Subscriber(topic,PoseStamped,self.__pose_callback__))
        topic = namespace+"/mavros/local_position/velocity_local"
        self.Subscribers.append(rospy.Subscriber(topic,TwistStamped,self.__velocity_callback__))
        topic = namespace+"/mavros/state"
        self.Subscribers.append(rospy.Subscriber(topic,State,self.__state_callback__))
        topic = namespace+"/mavros/extended_state"
        self.Subscribers.append(rospy.Subscriber(topic,ExtendedState,self.__extendedstate_callback__))

    ###########################################################################
    ### __create_publishers__() method - To be called in the constructor to
    ### create the publisher objects for the topic we desire to publish
    ###########################################################################
    def __create_publishers__(self):
        namespace = "/Drone_"+str(self.droneID)
        topic = namespace+"/mavros/setpoint_raw/attitude"
        self.att_pub = rospy.Publisher(topic, AttitudeTarget, queue_size=1)
        topic = namespace+"/mavros/setpoint_raw/local"
        self.pos_pub = rospy.Publisher(topic, PositionTarget, queue_size=1)
        self.att_set = AttitudeTarget()
        self.att_set.type_mask = int("00000111",2)
        self.pos_set = PositionTarget()
        self.pos_set.type_mask = int('100111111000',2)
        self.pos_set.coordinate_frame = 1

    ###########################################################################
    ### Subscriber callback definition
    ###########################################################################
    def __pose_callback__(self,data):
        p = data.pose.position
        self.p = np.array((p.x,p.y,p.z))
        o = data.pose.orientation
        self.r = R.from_quat((o.x,o.y,o.z,o.w))

    def __velocity_callback__(self,data):
        v = data.twist.linear
        self.v = np.array((v.x,v.y,v.z))

    def __state_callback__(self,data):
        if not self.is_["connected"] and data.connected:
            print("Drone "+repr(self.droneID)+" is now connected.")
        self.is_["connected"] = data.connected
        self.is_["armed"] = data.armed
        self.mode = data.mode

    def __extendedstate_callback__(self,data):
        self.is_["landed"] = False
        if data.landed_state == 1:
            self.is_["landed"] = True

    ###########################################################################
    ### send_arm_command() method - Sends an arm/disarm command, according to
    ### the Arm flag
    ###########################################################################
    def send_arm_command(self):
        namespace = "Drone_"+str(self.droneID)
        service = namespace+'/mavros/cmd/arming'
        rospy.wait_for_service(service)
        try:
            armService = rospy.ServiceProxy(service, mavros_msgs.srv.CommandBool)
            armService(True)
        except rospy.ServiceException as e:
            print("Service arming call failed: %s"%e)

    ###########################################################################
    ### send_arm_command() method - Sends an arm/disarm command, according to
    ### the Arm flag
    ###########################################################################
    def send_disarm_command(self, force=False):
        namespace = "Drone_"+str(self.droneID)
        try:
            service = namespace+'/mavros/cmd/command'
            rospy.wait_for_service(service)
            disarmService = rospy.ServiceProxy(service, mavros_msgs.srv.CommandLong)
            if force:
                disarmService(False,400,0,0,21196,0,0,0,0,0)
            else:
                disarmService(False,400,0,0,0,0,0,0,0,0)
        except rospy.ServiceException as e:
            print("Service disarm call failed: %s"%e)


    ###########################################################################
    ### send_mode() method - Sends a mode request
    ###########################################################################
    def set_mode(self,mode):
        namespace = "Drone_"+str(self.droneID)
        service = namespace+'/mavros/set_mode'
        rospy.wait_for_service(service)
        try:
            flightModeService = rospy.ServiceProxy(service, mavros_msgs.srv.SetMode)
            flightModeService(custom_mode=mode)
        except rospy.ServiceException as e:
               print("service set_mode call failed: %s. Mode could not be set."%e)

    ###########################################################################
    ### get_data() method - retrieve drone data
    ###########################################################################
    def get_data(self):
        return self.p, self.v, self.r

    ###########################################################################
    ### send_position_command() method - Sends a position setpoint
    ###########################################################################
    def send_position_command(self,p,yaw):
        self.pos_set.position.x = p[0]
        self.pos_set.position.y = p[1]
        self.pos_set.position.z = p[2]
        self.pos_set.yaw = yaw
        print(self.pos_set)
        self.pos_pub.publish(self.pos_set)

    ###########################################################################
    ### send_att_thrust_command() method - Sends an attitude setpoint
    ###########################################################################
    def send_att_thrust_command(self,thrust,orientation):
        o = orientation.as_quat()
        self.att_set.orientation.x = o[0]
        self.att_set.orientation.y = o[1]
        self.att_set.orientation.z = o[2]
        self.att_set.orientation.w = o[3]
        self.att_set.thrust = thrust
        self.att_pub.publish(self.att_set)

    ###########################################################################
    ### disconnect() method - This disconnects the communication. The instance
    ### becomes invalid
    ###########################################################################
    def disconnect(self):
        if self.droneID in Connection.droneIDs:
            [subscriber.unregister() for subscriber in self.Subscribers]
            self.att_pub.unregister()
            self.pos_pub.unregister()
            self.mavROS.send_signal(signal.SIGINT)
            Connection.droneIDs.remove(self.droneID)
            self.is_["connected"] = False
            if hasattr(self, 'vrpn'):
                self.vrpn.send_signal(signal.SIGINT)
            if hasattr(self, 'remap'):
                self.remap.send_signal(signal.SIGINT)

    ###########################################################################
    ### __del__() method - Class destructor
    ###########################################################################
    def __del__(self):
        if self.droneID in Connection.droneIDs:
            [subscriber.unregister() for subscriber in self.Subscribers]
            self.att_pub.unregister()
            self.pos_pub.unregister()
            self.mavROS.send_signal(signal.SIGINT)
            Connection.droneIDs.remove(self.droneID)
            self.is_["connected"] = False


###############################################################################
###     Class: DroneManager
### This class implements a GUI interface to the Drones. It contains a list
### of Drone objects to control. This is blocking.
###############################################################################
class DroneManager:

    drones = []
    obstacles = []

    def __init__(self, drones_,obstacles_=[], takeoff_alt_=1, log=True):
        DroneManager.drones = drones_
        DroneManager.obstacles = obstacles_
        self.takeoff_alt = takeoff_alt_
        self.log = log
        for drone in DroneManager.drones:
            drone.connect()
        if self.log:
            self.rate = rospy.Rate(Sampling_Frequency)
            self.running = True
            self.thread = Thread(target=self.__log_thread__)
            self.thread.start()
        self.GUI_window()

    def arm_all(self):
        for drone in DroneManager.drones:
            drone.arm()

    def disarm_all(self):
        for drone in DroneManager.drones:
            drone.disarm()

    def emergency_disarm_all(self):
        for drone in DroneManager.drones:
            drone.emergency_disarm()

    def hover_all(self):
        for drone in DroneManager.drones:
            drone.hover()

    def land_all(self):
        for drone in DroneManager.drones:
            drone.land()

    def takeoff_all(self):
        for drone in DroneManager.drones:
            drone.hover(self.takeoff_alt)

    def switch_all_to_controller(self):
        for drone in DroneManager.drones:
            drone.switch_to_controller()

    def quit(self):
        print("Disconnecting and exiting...")
        self.running = False
        for drone in DroneManager.drones:
            if not drone.connection.is_['landed']:
                drone.land()
            drone.disconnect()
        for obstacle in DroneManager.obstacles:
            obstacle.disconnect()
        DroneManager.drones = []
        DroneManager.obstacles = []
        self.root.destroy()

    def GUI_window(self):
        self.root = tk.Tk()
        self.f = tk.Frame(self.root)
        self.f.pack()
        self.buttons = {}
        self.buttons["arm"] = tk.Button(self.f, text="ARM", fg="black",
            bg="white", command=self.arm_all)
        self.buttons["takeoff"] = tk.Button(self.f, text="TAKEOFF", fg="black",
            bg="white",command=self.takeoff_all)
        self.buttons["hover"] = tk.Button(self.f, text="HOVER", fg="black",
            command=self.hover_all)
        self.buttons["land"] = tk.Button(self.f, text="LAND", fg="black",
            command=self.land_all)
        self.buttons["switch"] = tk.Button(self.f, text="SWITCH TO CONTROLLER",
            fg="black", command=self.switch_all_to_controller)
        self.buttons["disarm"] = tk.Button(self.f, text="DISARM", fg="black",
            command=self.disarm_all)
        self.buttons["quit"] = tk.Button(self.f, text="QUIT", fg="black",
            command=self.quit)
        self.buttons["emergency"] = tk.Button(self.f, text="EMERGENCY DISARM",
            fg="black", bg="red", command=self.emergency_disarm_all)
        for button in self.buttons.values():
            button.pack(side=tk.RIGHT)
        self.root.mainloop()

    def __log_thread__(self):
        while self.running:
            t = rospy.get_time()
            for drone in DroneManager.drones:
                if drone.log:
                    p,v,r,u = drone.p_,drone.v,drone.r,drone.u
                    yaw = r.as_euler('xyz')[2]
                    drone.logger.add(t,p,v,u,yaw,drone.controlling)
            for obstacle in DroneManager.obstacles:
                if obstacle.log:
                    p,v = obstacle.get_data()
                    obstacle.logger.add(t,p,v)
            self.rate.sleep()



###############################################################################
###     Class: Logger
### This class is used to log the data aquired during the simulation
###############################################################################
class Logger:

    IDs = []

    def __init__(self,ID):
        error = "Logger with given ID already exists"
        assert (ID not in Logger.IDs), error
        self.ID = ID
        Logger.IDs.append(self.ID)
        self.MAX_LOG = 1000
        self.log = {}
        self.log["Time"] = np.empty(0)
        self.log["Controlling"] = np.empty(0)
        self.log["Position"] = np.empty((3,0))
        self.log["Velocity"] = np.empty((3,0))
        self.log["Input"] = np.empty((3,0))
        self.log["Yaw"] = np.empty(0)
        self.counter = 0
        self.size = 0

    def add(self,t,p,v,u,yaw,controlling):
        if self.counter >= self.size:
            self.log["Time"] = np.append(self.log["Time"],np.empty(self.MAX_LOG))
            self.log["Controlling"] = np.append(self.log["Controlling"],np.empty(self.MAX_LOG,dtype=bool))
            self.log["Position"] = np.hstack((self.log["Position"],np.empty((3,self.MAX_LOG))))
            self.log["Velocity"] = np.hstack((self.log["Velocity"],np.empty((3,self.MAX_LOG))))
            self.log["Input"] = np.hstack((self.log["Input"],np.empty((3,self.MAX_LOG))))
            self.log["Yaw"] = np.append(self.log["Yaw"],np.empty(self.MAX_LOG))
            self.size += self.MAX_LOG
        self.log["Time"][self.counter] = t
        self.log["Controlling"][self.counter] = controlling
        self.log["Position"][:,self.counter] = p
        self.log["Velocity"][:,self.counter] = v
        self.log["Input"][:,self.counter] = u
        self.log["Yaw"][self.counter] = yaw
        self.counter += 1

    def save_data(self):
        self.log["Time"] = self.log["Time"][0:self.counter]
        self.log["Controlling"] = self.log["Controlling"][0:self.counter]
        self.log["Position"] = self.log["Position"][:,0:self.counter]
        self.log["Velocity"] = self.log["Velocity"][:,0:self.counter]
        self.log["Input"] = self.log["Input"][:,0:self.counter]
        self.log["Yaw"] = self.log["Yaw"][0:self.counter]
        dict = {}
        dict["Drone_"+str(self.ID)] = self.log
        savemat("./Saved_Data/Drone_"+str(self.ID)+".mat",dict,oned_as='column')
