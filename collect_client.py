import yaml
import carla
from sensor import *
from vehicle import Vehicle
from walker import Walker
import time
def get_location(location):
    return (location.x,location.y,location.z)
    
class CollectClient:
    def __init__(self,config_path):
        with open(config_path,'r') as f:
            self.config = yaml.load(f.read(),Loader=yaml.CLoader)
        self.client = carla.Client(**self.config["client"]["init"])
        self.client.set_timeout(self.config["client"]["time_out"])
        
        self.world = self.client.get_world()
        self.original_settings = self.world.get_settings()
        self.world.unload_map_layer(carla.MapLayer.ParkedVehicles)
        
        self.ego_vehicle = None
        self.sensors = None
        self.vehicles = None
        self.walkers = None
        self.trafficmanager = self.client.get_trafficmanager()

        try:
            self.generate()
        except Exception:
            self.destroy()
            time.sleep(0.5)
            raise Exception("Generate Fail!")
        print("Generate Success!")

    def generate(self):
        self.trafficmanager.set_synchronous_mode(True)
        self.trafficmanager.set_respawn_dormant_vehicles(True)
        self.settings = carla.WorldSettings(**self.config["world"]["init"])
        self.settings.synchronous_mode = True
        self.settings.no_rendering_mode = False
        self.world.apply_settings(self.settings)
        if self.config["world"]["weather_mode"] == "custom":
            self.weather = carla.WeatherParameters(**self.config["world"]["weather"])
            self.world.set_weather(self.weather)

        SpawnActor = carla.command.SpawnActor
        SetAutopilot = carla.command.SetAutopilot
        FutureActor = carla.command.FutureActor

        self.ego_vehicle = Vehicle(world=self.world,**self.config["ego_vehicle"]["init"])
        self.ego_vehicle.blueprint.set_attribute('role_name', 'hero')
        self.ego_vehicle.spawn_actor()

        self.vehicles = [Vehicle(world=self.world,**vehicle_config["init"]) for vehicle_config in self.config["vehicles"]]
        vehicles_batch = [SpawnActor(vehicle.blueprint,vehicle.transform)
                            .then(SetAutopilot(FutureActor, True, self.trafficmanager.get_port())) 
                            for vehicle in self.vehicles]
        for i,response in enumerate(self.client.apply_batch_sync(vehicles_batch)):
            if not response.error:
                self.vehicles[i].set_actor(response.actor_id)
            else:
                print(response.error)
        self.vehicles = list(filter(lambda vehicle:vehicle.get_actor(),self.vehicles))
        print(self.vehicles)
        for vehicle in self.vehicles:
            self.trafficmanager.set_path(vehicle.get_actor(),vehicle.path)

        self.walkers = [Walker(world=self.world,**walker_config["init"]) for walker_config in self.config["walkers"]]
        walkers_batch = [SpawnActor(walker.blueprint,walker.transform) for walker in self.walkers]
        for i,response in enumerate(self.client.apply_batch_sync(walkers_batch)):
            if not response.error:
                self.walkers[i].set_actor(response.actor_id)
            else:
                print(response.error)
        self.walkers = list(filter(lambda walker:walker.get_actor(),self.walkers))

        walker_controller_bp = self.world.get_blueprint_library().find('controller.ai.walker')
        walkers_controller_batch = [SpawnActor(walker_controller_bp,carla.Transform(),walker.get_actor()) for walker in self.walkers]
        for i,response in enumerate(self.client.apply_batch_sync(walkers_controller_batch)):
                    if not response.error:
                        self.walkers[i].set_controller(response.actor_id)
                    else:
                        print(response.error)
        for walker in self.walkers:
            walker.start()

        self.sensors = [Sensor(world=self.world, attach_to=self.ego_vehicle.get_actor(), **sensor_config["init"]) for sensor_config in self.config["sensors"]]
        sensors_batch = [SpawnActor(sensor.blueprint,sensor.transform) for sensor in self.sensors]
        for i,response in enumerate(self.client.apply_batch_sync(sensors_batch)):
            if not response.error:
                self.sensors[i].set_actor(response.actor_id)
            else:
                print(response.error)
        self.sensors = list(filter(lambda sensor:sensor.get_actor(),self.sensors))


    def run(self):
        try:
            for count in range(int(self.config["collect_time"]/self.settings.fixed_delta_seconds)):
                print("count:",count)
                self.tick(count)
        except Exception as err:
            print(err)
        finally:
            self.destroy()

    def tick(self,count):
        self.world.tick()
        if count % int(self.config["keyframe_time"]/self.settings.fixed_delta_seconds) == 0:
            ego_location = self.ego_vehicle.get_actor().get_transform().location
            ego_rotation = self.ego_vehicle.get_actor().get_transform().rotation
            print(ego_location,ego_rotation)
            for sensor in self.sensors:
                print("sensor_data:",len(sensor.get_data()))
                sensor.data_list.clear()
            for walker in self.walkers:
                if(self.is_visible(self.ego_vehicle,walker)):
                    print(walker.get_actor().get_transform().location)
            for vehicle in self.vehicles:
                if(self.is_visible(self.ego_vehicle,vehicle)):
                    print(vehicle.get_actor().get_transform().location)

    def destroy(self):
        if self.walkers is not None:
            for walker in self.walkers:
                walker.controller.stop()
                walker.destroy()
        if self.vehicles is not None:
            for vehicle in self.vehicles:
                vehicle.destroy()
        if self.sensors is not None:
            for sensor in self.sensors:
                sensor.destroy()
        if self.ego_vehicle is not None:
            self.ego_vehicle.destroy()
        self.trafficmanager.set_synchronous_mode(False)
        self.ego_vehicle = None
        self.sensors = None
        self.vehicles = None
        self.walkers = None
        self.world.apply_settings(self.original_settings)

    def is_visible(self,ego,target):
        print(get_location(ego.get_actor().get_transform().location),get_location(target.get_actor().get_transform().location))
        points =  self.world.cast_ray(ego.get_actor().get_transform().location,target.get_actor().get_transform().location)
        print([(get_location(point.location),point.label) for point in points])
        return True
