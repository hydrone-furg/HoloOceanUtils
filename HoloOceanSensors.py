import holoocean
import holoocean.agents
import holoocean.sensors
import holoocean.holooceanclient

class Sensors:
    def __init__(self,agent_name:str,agent_type:str) -> None:
        self.agent_name=agent_name
        self.agent_type=agent_type
        self.image_sonar=None
        self.image_sonar_config= {
                        "RangeBins": 256,
                        "AzimuthBins": 96,
                        "RangeMin": 0,
                        "RangeMax": 4,
                        "InitOctreeRange": 50,
                        "Elevation": 28,
                        "Azimuth": 28.8,
                        "AzimuthStreaks": -1,
                        "ScaleNoise": True,
                        "AddSigma": 0.15,
                        "MultSigma": 0.2,
                        "RangeSigma": 0.0,
                        "MultiPath": True,
						"ViewOctree": -1
                    }
        
        """
        self.image_sonar_config={
                "RangeBins":394,
                "AzimuthBins":768,
                "RangeMin": 0.5,
                "RangeMax": 10,
                "InitOctreeRange":20,
                "Elevation": 20,
                "Azimuth": 130,
                "AzimuthStreaks": -1,
                "ScaleNoise": True,
                "AddSigma": 0.05,
                "MultSigma": 0.05,
                "RangeSigma": 0.05,
                "MultiPath": True,
                "ViewRegion": True,
                "ViewOctree": -1
                }
        """
        self.location_sensor=None
        self.rotation_sensor=None

    def addPositionSensor(self)->None:
        self.location_sensor=holoocean.sensors.SensorDefinition(
            agent_name=self.agent_name,
            agent_type=self.agent_type,
            sensor_name="LocationSensor",
            sensor_type="LocationSensor",
            socket="Origin")
        
        self.rotation_sensor=holoocean.sensors.SensorDefinition(
            agent_name=self.agent_name,
            agent_type=self.agent_type,
            sensor_name="RotationSensor",
            sensor_type="RotationSensor",
            socket="Origin")
        
    def addImagingSonar(self)->None:
        self.image_sonar=holoocean.sensors.SensorDefinition(
            agent_name=self.agent_name,
            agent_type=self.agent_type,
            sensor_name="ImagingSonar",
            sensor_type="ImagingSonar",
            socket="SonarSocket",
            config=self.image_sonar_config)