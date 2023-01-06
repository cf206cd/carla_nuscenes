import carla
from sensor import *
from vehicle import Vehicle
from walker import Walker
from utils import get_token,get_rt,get_intrinsic
    
class CollectClient:
    def __init__(self,client_config):
        self.client = carla.Client(client_config["host"],client_config["ip"])
        self.client.set_timeout(client_config["time_out"])
        self.modality_dict = {
            'sensor.camera.rgb':'camera',
            'sensor.other.radar':'radar',
            'sensor.lidar.ray_cast':'lidar'
        }
        

    def generate_world(self,world_config):
        self.client.load_world(world_config["map_name"])
        self.world = self.client.get_world()
        self.original_settings = self.world.get_settings()
        self.world.unload_map_layer(carla.MapLayer.ParkedVehicles)
        self.ego_vehicle = None
        self.sensors = None
        self.vehicles = None
        self.walkers = None

        self.category_dict = {bp.id: ".".split(bp.id)[0] for bp in self.world.get_blueprint_library()}

        self.trafficmanager = self.client.get_trafficmanager()
        self.trafficmanager.set_synchronous_mode(True)
        self.trafficmanager.set_respawn_dormant_vehicles(True)
        self.settings = carla.WorldSettings(world_config["settings"])
        self.settings.synchronous_mode = True
        self.settings.no_rendering_mode = False
        self.world.apply_settings(self.settings)

    def generate_scene(self,scene_config):
        if scene_config["weather_mode"] == "custom":
            self.weather = carla.WeatherParameters(**scene_config["weather"])
            self.world.set_weather(self.weather)

        SpawnActor = carla.command.SpawnActor
        SetAutopilot = carla.command.SetAutopilot
        FutureActor = carla.command.FutureActor

        self.ego_vehicle = Vehicle(world=self.world,**scene_config["ego_vehicle"]["init"])
        self.ego_vehicle.blueprint.set_attribute('role_name', 'hero')
        self.ego_vehicle.spawn_actor()
        self.ego_vehicle.get_actor().set_autopilot()

        self.vehicles = [Vehicle(world=self.world,**vehicle_config["init"]) for vehicle_config in scene_config["vehicles"]]
        vehicles_batch = [SpawnActor(vehicle.blueprint,vehicle.transform)
                            .then(SetAutopilot(FutureActor, True, self.trafficmanager.get_port())) 
                            for vehicle in self.vehicles]
        for i,response in enumerate(self.client.apply_batch_sync(vehicles_batch)):
            if not response.error:
                self.vehicles[i].set_actor(response.actor_id)
            else:
                print(response.error)
        self.vehicles = list(filter(lambda vehicle:vehicle.get_actor(),self.vehicles))

        for vehicle in self.vehicles:
            self.trafficmanager.set_path(vehicle.get_actor(),vehicle.path)

        self.walkers = [Walker(world=self.world,**walker_config["init"]) for walker_config in scene_config["walkers"]]
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
        self.world.tick()
        for walker in self.walkers:
            walker.start()
        self.sensors = [Sensor(world=self.world, attach_to=self.ego_vehicle.get_actor(), **sensor_config["init"]) for sensor_config in scene_config["sensors"]]
        sensors_batch = [SpawnActor(sensor.blueprint,sensor.transform,sensor.attach_to) for sensor in self.sensors]
        for i,response in enumerate(self.client.apply_batch_sync(sensors_batch)):
            if not response.error:
                self.sensors[i].set_actor(response.actor_id)
            else:
                print(response.error)
        self.sensors = list(filter(lambda sensor:sensor.get_actor(),self.sensors))

    def tick(self):
        self.world.tick()

    def destroy_scene(self):
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


    def destroy_world(self):
        self.trafficmanager.set_synchronous_mode(False)
        self.ego_vehicle = None
        self.sensors = None
        self.vehicles = None
        self.walkers = None
        self.world.apply_settings(self.original_settings)

    def is_invisible(self,ego,target):
        ego_bbox_center = ego.get_location()
        target_bbox_center = target.get_location()
        points =  self.world.cast_ray(ego_bbox_center,target_bbox_center)
        points = list(filter(lambda point:not ego.get_actor().bounding_box.contains(point.location,ego.get_actor().get_transform()) 
                            and not target.get_actor().bounding_box.contains(point.location,target.get_actor().get_transform()),points))
        return points is True

    def get_calibrated_sensor(self,sensor):
        sensor_token = get_token("sensor",sensor.name)
        channel = sensor.name
        rotation,translation = get_rt(sensor.transform)
        if sensor.bp_name == "sensor.camera.rgb":
            intrinsic = get_intrinsic(sensor.fov,sensor.image_size_x,sensor.image_size_y)
        else:
            intrinsic = []
        return sensor_token,channel,translation,rotation,intrinsic
        
    def get_ego_pose(self,sample_data):
        timestamp = sample_data[1].timestamp
        rotation,translation = get_rt(sample_data[0])
        return timestamp,translation,rotation
    
    def get_sample_data(self,sample_data):
        height = 0
        width = 0 
        if isinstance(sample_data,carla.Image):
            height = sample_data.height
            width = sample_data.width
        return height,width

    def get_instance(self,scene_id,instance):
        category_token = get_token("category",self.category_dict[instance.bp_name])
        id = hash((scene_id,instance.get_actor().id))
        return category_token,id

    def get_sample_annotation(self,scene_id,instance):
        instance_token = get_token("token",hash((scene_id,instance.get_actor().id)))
        visibility_token = str(self.get_visibility(instance))
        attribute_tokens = [get_token("attribute",attribute) for attribute in self.get_attributes(instance)]
        rotation,translation = get_rt(instance.get_transform())
        size = instance.get_size()
        num_lidar_pts = 4#todo
        num_radar_pts = 4#todo
        return instance_token,visibility_token,attribute_tokens,translation,rotation,size,num_lidar_pts,num_radar_pts


    def get_visibility(self,instance):
        return 4#todo

    def get_attributes(self,instance):
        return "vehicle.parked"#todo