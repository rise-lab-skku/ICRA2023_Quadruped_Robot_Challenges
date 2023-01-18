from distutils.command.clean import clean
import string
from simApi import sim

import rospy
import rospkg
import tf
from geometry_msgs.msg import Pose
import time
import os 
import sys
import yaml

ros_root = rospkg.get_ros_root()
r = rospkg.RosPack()
PKG_NAME = "ICRA2023_Quadruped_Competition"
PKG_PATH = r.get_path(PKG_NAME)
SCENE_PATH = PKG_PATH + '/coppelia_sim/'

class URDFGenerator:    

    def __init__(self, name, path):      
        # init parameters
        self.package_path = path
        self.name = name
        self.file_path = self.package_path +'/urdf/map.urdf.xacro'
 

    def generate(self, object_data):
        '''
        Make file in specific directory
        '''
        result = True
        # open file
        f = open(self.file_path, 'w+')

        # header
        f.write('<?xml version="1.0" ?>\n')
        f.write('<robot xmlns:xacro="http://ros.org/wiki/xacro" name="'+self.name+'">\n\n')
        f.write('\t<link name="world"/>\n\n')
        f.write('\t<xacro:include filename="$(find ICRA2023_Quadruped_Competition)/urdf/obstacles/materials.xacro" />\n\n')

        # contents
        for obj_class in object_data.keys():
            self.add_object(f, object_data[obj_class])

        # close
        f.write("</robot>")
        f.close()
        return True

    def add_object(self, file, object_list):
        if len(object_list)>0:
            obj_name = object_list[0].name

            if obj_name == '4_inch_solid_wood_block':
                obj_name = 'solid_wood_block'
            elif obj_name == 'Akro_Mils_37278':
                obj_name = 'Akro_mils'                
            elif obj_name == 'Plastic_Storage_Crate_visual':
                obj_name = 'Plastic_Storage_Crate'
            
            # include object xacro
            file.write('\t<!-- ' + obj_name + ' -->\n')
            file.write('\t<xacro:include filename="$(find '+ PKG_NAME + ')/urdf/obstacles/' + obj_name + '.urdf.xacro"/>\n\n')
            
            for obj in object_list:

                position = str(obj.position[0]) + ' ' + str(obj.position[1]) + ' ' + str(obj.position[2])

                quaternioin = obj.orientation
                # convert quaternion to rpy
                rpy = self.quaternion_to_euler(quaternioin)
                orientation = str(rpy[0]) + ' ' + str(rpy[1]) + ' ' + str(rpy[2])      

                file.write('\t<xacro:' + obj_name + ' parent="world" handle="' + str(obj.handle) + '" position="' + position + '" orientation="' + orientation + '"/>\n')
    
    def quaternion_to_euler(self, quaternion):
        euler = tf.transformations.euler_from_quaternion(quaternion)
        roll = euler[0]
        pitch = euler[1]
        yaw = euler[2]
        return [roll, pitch, yaw]

        
class SceneObject:
    def __init__(self, name, handle):
        self.name = name
        self.handle = handle
        self.position = []
        self.orientation = []
        self.size = []
        self.type = None
        self.parent = None
        self.child = None

    def __str__(self) -> str:
        name = self.name
        handle = self.handle
        size = self.size
        return (f'Name:{name}, Handle:{handle}, Size:{size[0], size[1], size[2]}, position:{self.position}, orientation:{self.orientation}')



class Client:
    def __init__(self):
        self.intSignalName='legacyRemoteApiStepCounter'        
        self.stepCounter=0
        self.lastImageAcquisitionTime=-1
        self.runInSynchronousMode = False
        self.sim_running = False
        self.start_time = None
        self.dt = 0.02  # second
        self.scene_objects = {}
        self.object_handles = []
        self.copied_objects = {}

        sim.simxFinish(-1) # just in case, close all opened connections
        self.id=sim.simxStart('127.0.0.1',19997,True,True,5000,5) # Connect to CoppeliaSim

        if self.id!=-1:
            print ('Connected to remote API server')
            sim.simxSynchronous(self.id, self.runInSynchronousMode)     
        else:
            print('Connection failed...')

    def __del__(self):          
        client.stopSimulation()
        # time.sleep(0.1)
        # client.closeScene()
        # sim.simxFinish(self.id)
        print('Disconnected from the CoppeliaSim')

    def startSimulation(self):    
        # Start streaming client.intSignalName integer signal, that signals when a step is finished:
        sim.simxGetIntegerSignal(client.id,client.intSignalName,sim.simx_opmode_streaming)   

        res = sim.simxStartSimulation(client.id,sim.simx_opmode_oneshot)        
        self.start_time=time.time()

        if res != -1:
            self.sim_running = True
            print("Start Simulation.")   
            time.sleep(1.0)
        
    def closeScene(self):
        res = sim.simxCloseScene(self.id, sim.simx_opmode_blocking)
        res = sim.simxCloseScene(self.id, sim.simx_opmode_blocking) # for sync mode
        if res != -1:
            print("Scene is closed...")

    def stopSimulation(self):        
        '''
        CoppeliaSim simulation stop
        '''
        if self.id != -1:
            # stop the simulation:
            res = sim.simxStopSimulation(self.id,sim.simx_opmode_blocking)
            res, ping = sim.simxGetPingTime(self.id)

        print("Stop Simulation.")           
        self.sim_running = False
        time.sleep(1.0)

    def loadScene(self, scene_dir):
        '''
        load a CoppeliaSim simulation scene
        '''
        # check scene and model files
        if not os.path.exists(scene_dir):
            rospy.logerr('Invaild path!: ' + scene_dir)
            return -1
            
        # stop simulation first
        self.stopSimulation()

        res = sim.simxLoadScene(self.id, str(scene_dir), 1, sim.simx_opmode_blocking)
        if res != -1:  
            # set time step
            #sim.simxSetFloatParam(self.id, sim.sim_floatparam_simulation_time_step, self.dt, sim.simx_opmode_blocking)        
             
            print("Scene is loaded.") 
        else:
            print("Scene loading is failed...")


        return 0

    def getData(self):
        
        return 

    def updateSceneObjects(self, opmode=sim.simx_opmode_blocking):
        # get shapetype objects
        returnCode, handles, _, _, names = sim.simxGetObjectGroupData(self.id, 
                                                                objectType=sim.sim_object_shape_type,
                                                                dataType=0,
                                                                operationMode=opmode)

        self.loadObjects(handles, names)

        return returnCode, handles, names

    def updateSceneObjectsFromScript(self, object_list=[], opmode=sim.simx_opmode_blocking):
        # get shapetype objects from script
        if not self.sim_running:
            self.startSimulation()
        if self.sim_running:
            inputInts=[]
            inputFloats=[]
            inputStrings=[]
            inputBuffer=bytearray()
            res,handles,sizes,names,_= sim.simxCallScriptFunction(self.id,'RemoteScript',
                    sim.sim_scripttype_childscript,'getSceneObjects',inputInts,inputFloats,inputStrings,
                    inputBuffer,sim.simx_opmode_blocking)
            # print(handles)
            # print(names)
            self.loadObjects(handles, names, sizes, object_list, only_parents=False)
        return self.scene_objects

    def callScriptFunction(self, opmode=sim.simx_opmode_blocking):
        if self.sim_running:
            inputInts=[1,2,3]
            inputFloats=[53.21,17.39]
            inputStrings=['Hello','world!']
            inputBuffer=bytearray()
            inputBuffer.append(78)
            inputBuffer.append(42)
            res,retInts,retFloats,retStrings,retBuffer=sim.simxCallScriptFunction(self.id,'RemoteScript',
                sim.sim_scripttype_childscript,'testFunc',inputInts,inputFloats,inputStrings,
                inputBuffer,sim.simx_opmode_blocking)
            print(res)
            print (retInts)
            print (retFloats)
            print (retStrings)
            print (retBuffer)

    def loadObjects(self, handles, names, data=None, object_list=[], opmode=sim.simx_opmode_blocking, only_parents=True):
        print('Updating %d scene objects...' %(len(handles)))
        if len(handles) == len(names):
            obj_num = len(handles)
            for i in range(obj_num):         
                if only_parents:   
                    return_code, parent_handle = sim.simxGetObjectParent(self.id, handles[i], opmode)

                    # get only top-level objects(no child)
                    if parent_handle == -1:
                        obj = SceneObject(names[i], handles[i])
                        if data != None:
                            obj.size = data[3*i:3*i+3]
                        self.scene_objects[names[i]] = obj
                        self.scene_objects[handles[i]] = obj
                else:
                    obj = SceneObject(names[i], handles[i])   
                    if data != None:
                        # every 10th idx
                        obj.size = data[10*i:10*i+3]
                        obj.position = data[10*i+3:10*i+6]
                        obj.orientation = data[10*i+6:10*i+10]
                    
                    # if dict has no key add new list
                    if object_list !=[] and names[i] in object_list and handles[i] not in self.object_handles:
                        self.object_handles.append(handles[i])
                        if names[i] not in self.scene_objects:
                            self.scene_objects[names[i]] = [obj]
                        else:
                            self.scene_objects[names[i]].append(obj)        
        else:
            print('ERROR: Cannot match object names and handles.')    

    def addObject(self, handle, name):
        if name not in self.scene_objects:
            obj = SceneObject(name, handle)   
            self.scene_objects[name] = obj
            self.scene_objects[handle] = obj

    def removeObject(self, handle, name):
        if name in self.scene_objects:
            self.scene_objects.pop(name)
        if handle in self.scene_objects:
            self.scene_objects.pop(handle)

    def getObjectName(self, handle):        
        if handle in self.scene_objects:
            return self.scene_objects[handle].name
        else:       
            return -1

    def getObjectHandle(self, name):    
        if name in self.scene_objects:
            return self.scene_objects[name].handle 
        else:
            res,handle = sim.simxGetObjectHandle(self.id, name, operationMode=sim.simx_opmode_blocking)
            if res != -1:
                return handle
        return -1

    def getObjectPosition(self, name=None, handle=None):
        res,position = sim.simxGetObjectPosition(self.id, handle, -1, operationMode=sim.simx_opmode_blocking)
        return position

    def getObjectOrientation(self, name=None, handle=None):
        res,orientation = sim.simxGetObjectQuaternion(self.id, handle, -1, operationMode=sim.simx_opmode_blocking)
        return orientation

    def renameObject(self, name=None, handle=None):
        pass
    
    def setObjectPosition(self, name=None, handle=None, position=[0,0,0]):

        sim.simxSetObjectPosition(self.id, handle, -1, position, operationMode=sim.simx_opmode_oneshot)

    def setObjectOrientation(self, name=None, handle=None, orientation=[0,0,0,1]):

        sim.simxSetObjectQuaternion(self.id, handle, -1, orientation, operationMode=sim.simx_opmode_oneshot)

    def copyPasteObject(self, names=[], handles=[], position=[0,0,0], orientation=[0,0,0,1]):
        object_handles = []
        if len(names)>0:
            for n in names:
                object_handles.append(self.getObjectHandle(n))
        elif len(handles)>0:
            object_handles = handles
        else:
            return -1, []
        
        for obj_handle in object_handles:
            res,new_object_handles = sim.simxCopyPasteObjects(self.id, [obj_handle], sim.simx_opmode_blocking)

            if res != -1:
                for i,h in enumerate(new_object_handles):
                    original_handle = object_handles[i]
                    original_name = self.scene_objects[original_handle].name
                    idx = 0
                    if original_name not in self.copied_objects:
                        self.copied_objects[original_name] = []
                    
                    idx = len(self.copied_objects[original_name])
                    copied_name = original_name + '_#' + str(idx)
                    obj = SceneObject(copied_name, h)
                    obj.size = self.scene_objects[original_name].size
                    self.copied_objects[original_name].append(obj)
                    
                    # set pose
                    self.setObjectPosition(handle=h, position=[0,0,obj.size[2]/2.0])
                    self.setObjectOrientation(handle=h, orientation=orientation)

if __name__ == '__main__':
    rospy.init_node('coppelia_interface')

    # loop freqency
    loop_freq = 20
    coppelia_ver = '4.4'
    # if os.environ.get('COPPELIASIM_VERSION') is not None:
    #     coppelia_ver = os.environ["COPPELIASIM_VERSION"].replace("V","").replace("_",".").replace(".0","")

    ttt_dir = SCENE_PATH + 'v' + coppelia_ver +'/tablewares.ttt'

    client = Client()    
    client.startSimulation()

    # object lists
    obj_list = ['New_600x400_2way_Wooden_Pallet', 
                'Plywood_wall', 
                'New_EURO_EPAL_Pallet', 
                '4_inch_solid_wood_block', 
                'Plastic_Curb_Ramp', 
                'Nisopa_Kerb_Ramp',
                'seat_pad', 
                'Plywood_3mm_sheet', 
                'Plywood_16mm_sheet', 
                'Akro_mils', 
                'Plastic_Storage_Crate_visual']

    # load objects
    obstacles = client.updateSceneObjectsFromScript(obj_list)
    
    for obj in obj_list:
        obj_class = obj
        print(obj_class, len(obstacles[obj_class]))

    
    # create urdf.xacro
    urdf_generator = URDFGenerator('competition_map', PKG_PATH)
    urdf_generator.generate(obstacles)
    
        