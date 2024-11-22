import holoocean
import holoocean.agents
import holoocean.sensors
import holoocean.holooceanclient
import numpy as np
import os
import json
from HoloOceanVehicles import AUV
from warnings import warn

class Scenario:
    def __init__(self,name:str,world:str,package_name:str,ticks_per_sec:int) -> None:
        
        self.cfg={
            "name": name,
            "world": world,
            "package_name":package_name,
            "ticks_per_sec": ticks_per_sec,
            "frames_per_sec": True,
            "octree_min": 0.02,
            "octree_max": 5,
            "agents":[]
        }
    
    def addAgent(self, agent)->None:
        self.cfg["agents"].append(agent) 
        pass

class Mission():
    def __init__(self,mission_data:list,mission_id:int):
        self.mission_id=mission_id
        self.mission_data=mission_data
        
        self.mission_waypoints=[]
        self.number_of_waypoints:int=0
        self.reached_waypoints:int=0
        self.actual_waypoint=[]
        self.distance_tresh_hold=0.1
        self.angle_tresh_hold=2.0

    def createWaypoints(self)->None:

        start_location=[float(self.mission_data[2]),-1*float(self.mission_data[3]),float(self.mission_data[4])]
        end_z=(float(self.mission_data[4])+float(self.mission_data[5]))

        if self.mission_id==1:
            angles=np.linspace(0,350,36)
            headings=np.concatenate((np.linspace(180,350,18),np.linspace(0,170,18)), axis=None)
            elevation=np.arange(-2.5,end_z,0.3 )
            for z in elevation:
                for angle, heading in zip(angles,headings):
                    x=2*np.cos(np.deg2rad(angle))+start_location[0]
                    y=2*np.sin(np.deg2rad(angle))+start_location[1]
                    self.mission_waypoints.append([x,y,z,0,0,heading])
            self.number_of_waypoints=len(self.mission_waypoints)
            self.actual_waypoint=self.mission_waypoints[self.reached_waypoints]
        
        if self.mission_id==2:
            angles=np.linspace(0,350,36)
            radious=np.linspace(2,1,3)
            headings=np.concatenate((np.linspace(180,350,18),np.linspace(0,170,18)), axis=None)
            for r in radious:
                for angle, heading in zip(angles,headings):
                    x=r*np.cos(np.deg2rad(angle))+start_location[0]
                    y=r*np.sin(np.deg2rad(angle))+start_location[1]
                    self.mission_waypoints.append([x,y,end_z+1,0,0,heading])
            
            self.number_of_waypoints=len(self.mission_waypoints)
            self.actual_waypoint=self.mission_waypoints[self.reached_waypoints]

        if self.mission_id==3:
            elevation=np.arange(-2.5,end_z,0.3 )
            for i,z in enumerate(elevation):
                if i==0 or i%2==0:
                    angles=np.linspace(90,270,18)
                    headings=np.concatenate((np.linspace(270,350,9),np.linspace(0,90,9)), axis=None)
                    for angle, heading in zip(angles,headings):
                        x=2*np.cos(np.deg2rad(angle))+start_location[0]
                        y=2*np.sin(np.deg2rad(angle))+start_location[1]
                        self.mission_waypoints.append([x,y,z,0,0,heading])
                else:
                    angles=np.linspace(270,90,18)
                    headings=np.concatenate((np.linspace(90,0,9),np.linspace(350,270,9)), axis=None)
                    for angle, heading in zip(angles,headings):
                        x=2*np.cos(np.deg2rad(angle))+start_location[0]
                        y=2*np.sin(np.deg2rad(angle))+start_location[1]
                        self.mission_waypoints.append([x,y,z,0,0,heading])       
            self.number_of_waypoints=len(self.mission_waypoints)
            self.actual_waypoint=self.mission_waypoints[self.reached_waypoints]

        if self.mission_id==4:
            radious=np.linspace(2,1,3)
            for i,r in enumerate(radious):
                if i==0 or i%2==0:
                    angles=np.linspace(90,270,18)
                    headings=np.concatenate((np.linspace(270,350,9),np.linspace(0,90,9)), axis=None)
                    for angle, heading in zip(angles,headings):
                        x=r*np.cos(np.deg2rad(angle))+start_location[0]
                        y=r*np.sin(np.deg2rad(angle))+start_location[1]
                        self.mission_waypoints.append([x,y,end_z+1,0,0,heading])
                else:
                    angles=np.linspace(270,90,18)
                    headings=np.concatenate((np.linspace(90,0,9),np.linspace(350,270,9)), axis=None)
                    for angle, heading in zip(angles,headings):
                        x=r*np.cos(np.deg2rad(angle))+start_location[0]
                        y=r*np.sin(np.deg2rad(angle))+start_location[1]
                        self.mission_waypoints.append([x,y,end_z+1,0,0,heading])       
            self.number_of_waypoints=len(self.mission_waypoints)
            self.actual_waypoint=self.mission_waypoints[self.reached_waypoints]

    def saveState(self,auv):
        self.mission_waypoints=auv.waypoints[auv.reached_waypoints:]

    def endMission():
        pass

    def restartMission(self):
        pass

    def start(self):
        data=self.mission_data
        mission_id=self.mission_id
        self.createWaypoints()

        scenario = Scenario("Imaging_Sonar_Dataset","64-tank-Map-"+str(mission_id),"Dataset-world",200)

        auv = AUV(id=str(data[0]),location=self.actual_waypoint[0:3],rotation=self.actual_waypoint[3:],mission=mission_id,waypoints=self.mission_waypoints)
        #auv.reached_waypoints=self.reached_waypoints
        auv.addSonarImaging(hz=10,RangeBins=256,AzimuthBins=96,RangeMin=0,RangeMax=4,Elevation=28,Azimuth=28.8,AzimuthStreaks=-1,ScaleNoise=True,AddSigma=0.15,
                            MultSigma=0.2,RangeSigma=0.0,MultiPath=False,ViewOctree=-1)
        """
        "configuration": {
                        "RangeBins": 512,
                        "AzimuthBins": 96,
                        "RangeMin": 0,
                        "RangeMax": 8,
                        "InitOctreeRange": 50,
                        "Elevation": 28,
                        "Azimuth": 28.8,
                        "AzimuthStreaks": -1,
                        "ScaleNoise": true,
                        "AddSigma": 0.15,
                        "MultSigma": 0.2,
                        "RangeSigma": 0.0,
                        "MultiPath": true,
						"ViewOctree": -1
                    }
        """
        auv.addSensor("LocationSensor","Origin")
        auv.addSensor("RotationSensor","Origin")
        auv.addSensor("PoseSensor","Origin",[0,0,0])
        auv.imageViwer()
        scenario.addAgent(auv.agent)

        with open("Config.json",'w') as fp:
            json.dump(scenario.cfg, fp)
            os.system('mv '+'Config.json'+' '+auv.root_folder+'/'+auv.files_folder)

        env=holoocean.make(scenario_cfg=scenario.cfg,verbose=False)
        env.reset
        
        for l in self.mission_waypoints:
            env.draw_point([l[0], l[1], l[2]],[0,255,0], lifetime=0)
        #start Simulation

        env.move_viewport([float(data[2]),-1*float(data[3]),6],[0,0,180])

        state=env.tick()
        auv.updateState(state)

        while not auv.fineshedMission():
            warn('Passing!')
            pass
            #state=env.tick()
            #auv.updateState(state)
            #env.act(auv.name,auv.command)


        print("Finished Mission "+data[0])
        os.system("killall -e Holodeck")
    