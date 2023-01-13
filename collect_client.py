import carla
from sensor import *
from vehicle import Vehicle
from walker import Walker
import math
from utils import get_token,get_nuscenes_rt,get_intrinsic,transform_timestamp
    
class CollectClient:
    def __init__(self,client_config):
        self.client = carla.Client(client_config["host"],client_config["port"])
        self.client.set_timeout(client_config["time_out"])

    def generate_world(self,world_config):
        print("generate world start!")
        self.client.load_world(world_config["map_name"])
        self.world = self.client.get_world()
        self.original_settings = self.world.get_settings()
        self.world.unload_map_layer(carla.MapLayer.ParkedVehicles)
        self.ego_vehicle = None
        self.sensors = None
        self.vehicles = None
        self.walkers = None

        get_category = lambda bp: "vehicle.car" if bp.id.split(".")[0] == "vehicle" else "human.pedestrian.adult" if bp.id.split(".")[0] == "walker" else None
        self.category_dict = {bp.id: get_category(bp) for bp in self.world.get_blueprint_library()}
        get_attribute = lambda bp: ["vehicle.moving"] if bp.id.split(".")[0] == "vehicle" else ["pedestrian.moving"] if bp.id.split(".")[0] == "walker" else None
        self.attribute_dict = {bp.id: get_attribute(bp) for bp in self.world.get_blueprint_library()}

        self.trafficmanager = self.client.get_trafficmanager()
        self.trafficmanager.set_synchronous_mode(True)
        self.trafficmanager.set_respawn_dormant_vehicles(True)
        self.settings = carla.WorldSettings(**world_config["settings"])
        self.settings.synchronous_mode = True
        self.settings.no_rendering_mode = False
        self.world.apply_settings(self.settings)
        print("generate world success!")

    def generate_scene(self,scene_config):
        print("generate scene start!")
        if scene_config["weather_mode"] == "custom":
            self.weather = carla.WeatherParameters(**scene_config["weather"])
            self.world.set_weather(self.weather)

        SpawnActor = carla.command.SpawnActor
        SetAutopilot = carla.command.SetAutopilot
        FutureActor = carla.command.FutureActor

        self.ego_vehicle = Vehicle(world=self.world,**scene_config["ego_vehicle"])
        self.ego_vehicle.blueprint.set_attribute('role_name', 'hero')
        self.ego_vehicle.spawn_actor()
        self.ego_vehicle.get_actor().set_autopilot()

        self.vehicles = [Vehicle(world=self.world,**vehicle_config) for vehicle_config in scene_config["vehicles"]]
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

        self.walkers = [Walker(world=self.world,**walker_config) for walker_config in scene_config["walkers"]]
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

        self.sensors = [Sensor(world=self.world, attach_to=self.ego_vehicle.get_actor(), **sensor_config) for sensor_config in scene_config["calibrated_sensors"]["sensors"]]
        sensors_batch = [SpawnActor(sensor.blueprint,sensor.transform,sensor.attach_to) for sensor in self.sensors]
        for i,response in enumerate(self.client.apply_batch_sync(sensors_batch)):
            if not response.error:
                self.sensors[i].set_actor(response.actor_id)
            else:
                print(response.error)
        self.sensors = list(filter(lambda sensor:sensor.get_actor(),self.sensors))
        print("generate scene success!")

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

    def get_calibrated_sensor(self,sensor):
        sensor_token = get_token("sensor",sensor.name)
        channel = sensor.name
        if sensor.bp_name == "sensor.camera.rgb":
            intrinsic = get_intrinsic(float(sensor.get_actor().attributes["fov"]),
                            float(sensor.get_actor().attributes["image_size_x"]),
                            float(sensor.get_actor().attributes["image_size_y"])).tolist()
            rotation,translation = get_nuscenes_rt(sensor.transform,"zxy")
        else:
            intrinsic = []
            rotation,translation = get_nuscenes_rt(sensor.transform)
        return sensor_token,channel,translation,rotation,intrinsic
        
    def get_ego_pose(self,sample_data):
        timestamp = transform_timestamp(sample_data[1].timestamp)
        rotation,translation = get_nuscenes_rt(sample_data[0])
        return timestamp,translation,rotation
    
    def get_sample_data(self,sample_data):
        height = 0
        width = 0
        if isinstance(sample_data[1],carla.Image):
            height = sample_data[1].height
            width = sample_data[1].width
        return sample_data,height,width

    def get_sample(self):
        return (transform_timestamp(self.world.get_snapshot().timestamp.elapsed_seconds),)

    def get_instance(self,scene_id,instance):
        category_token = get_token("category",self.category_dict[instance.blueprint.id])
        id = hash((scene_id,instance.get_actor().id))
        return category_token,id

    def get_sample_annotation(self,scene_id,instance):
        instance_token = get_token("instance",hash((scene_id,instance.get_actor().id)))
        visibility_token = str(self.get_visibility(instance))
        attribute_tokens = [get_token("attribute",attribute) for attribute in self.get_attributes(instance)]
        rotation,translation = get_nuscenes_rt(instance.get_transform())
        size = [instance.get_size().y,instance.get_size().x,instance.get_size().z]#xyz to whl
        num_lidar_pts = 0
        num_radar_pts = 0
        for sensor in self.sensors:
            if sensor.bp_name == 'sensor.other.radar':
                num_lidar_pts += self.get_num_lidar_pts(instance,sensor.get_last_data(),sensor.get_transform())
            elif sensor.bp_name == 'sensor.lidar.ray_cast':
                num_radar_pts += self.get_num_radar_pts(instance,sensor.get_last_data(),sensor.get_transform())
        return instance_token,visibility_token,attribute_tokens,translation,rotation,size,num_lidar_pts,num_radar_pts

    def get_visibility(self,instance):
        ego_position = self.ego_vehicle.get_transform().location
        ego_position.z += self.ego_vehicle.get_size().z*0.5
        instance_position = instance.get_transform().location
        visible_point_count1 = 0
        visible_point_count2 = 0
        for i in range(5):
            size = instance.get_size()
            size.z = 0
            check_point = instance_position-(i-2)*size*0.5
            ray_points =  self.world.cast_ray(ego_position,check_point)
            points = list(filter(lambda point:not self.ego_vehicle.get_actor().bounding_box.contains(point.location,self.ego_vehicle.get_actor().get_transform()) 
                                and not instance.get_actor().bounding_box.contains(point.location,instance.get_actor().get_transform()) 
                                and point.label is not carla.libcarla.CityObjectLabel.NONE,ray_points))
            if not points:
                visible_point_count1+=1
            size.x = -size.x
            check_point = instance_position-(i-2)*size*0.5
            ray_points =  self.world.cast_ray(ego_position,check_point)
            points = list(filter(lambda point:not self.ego_vehicle.get_actor().bounding_box.contains(point.location,self.ego_vehicle.get_actor().get_transform()) 
                                and not instance.get_actor().bounding_box.contains(point.location,instance.get_actor().get_transform()) 
                                and point.label is not carla.libcarla.CityObjectLabel.NONE,ray_points))
            if not points:
                visible_point_count2+=1
        visible_point_count = max(visible_point_count1,visible_point_count2)
        visibility_dict = {0:0,1:1,2:1,3:2,4:3,5:4}
        return visibility_dict[visible_point_count]

    def get_attributes(self,instance):
        return self.attribute_dict[instance.bp_name]

    def get_num_lidar_pts(self,instance,lidar_data,lidar_transform):#to check
        num_lidar_pts = 0
        if lidar_data is not None:
            for data in lidar_data[1]:
                point = lidar_transform.transform(data.point)
                if instance.get_actor().bounding_box.contains(point,instance.get_actor().get_transform()):
                    num_lidar_pts+=1
        return num_lidar_pts

    def get_num_radar_pts(self,instance,radar_data,radar_transform):#to check
        num_radar_pts = 0
        if radar_data is not None:
            for data in radar_data[1]:
                point = carla.Location(data.depth*math.cos(data.altitude)*math.cos(data.azimuth),
                        data.depth*math.sin(data.altitude)*math.cos(data.azimuth),
                        data.depth*math.sin(data.azimuth)
                        )
                if instance.get_actor().bounding_box.contains(radar_transform.transform(point),instance.get_actor().get_transform()):
                    num_radar_pts+=1
        print("num_radar_pts",num_radar_pts)
        return num_radar_pts

    