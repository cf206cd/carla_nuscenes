from collect_client import CollectClient
from dataset import Dataset
import yaml
from yamlinclude import YamlIncludeConstructor
import traceback

YamlIncludeConstructor.add_to_loader_class(loader_class=yaml.FullLoader)
class Runner:
    def __init__(self,config_path):
        with open(config_path,'r') as f:
            self.config = yaml.load(f.read(),Loader=yaml.FullLoader)
        self.dataset = Dataset(**self.config["dataset"])
        self.collect_client = CollectClient(self.config["client"])

    def generate_full_dataset(self):
        for sensor in self.config["sensors"]:
            self.dataset.update_sensor(sensor["name"],sensor["modality"])
        for category in self.config["categories"]:
            self.dataset.update_category(category["name"],category["description"])
        for attribute in self.config["attributes"]:
            self.attribute_token = self.dataset.update_attribute(attribute["name"],category["description"])
        for visibility in self.config["visibility"]:
            self.visibility = self.dataset.update_visibility(visibility["description"],visibility["level"])

        for world_config in self.config["worlds"]:
            try:
                self.collect_client.generate_world(world_config)
                map_token = self.dataset.update_map(world_config["map_name"],world_config["map_category"])
                for capture_config in world_config["captures"]:
                    log_token = self.dataset.update_log(map_token,capture_config["date"],capture_config["time"],
                                            capture_config["timezone"],capture_config["vehicle"],capture_config["location"])
                    for scene_id,scene_config in enumerate(capture_config["scenes"]):
                        self._add_one_scene(log_token,scene_id,scene_config)
            except:
                traceback.print_exc()
            finally:
                self.collect_client.destroy_world()    
        self.dataset.save()
    def add_one_scene(self):
        pass#todo

    def _add_one_scene(self,log_token,scene_id,scene_config):
        try:
            calibrated_sensors_token = {}
            samples_data_token = {}
            instances_token = {}
            samples_annotation_token = {}

            self.collect_client.generate_scene(scene_config)
            scene_token = self.dataset.update_scene(log_token,scene_id,scene_config["description"])

            for instance in self.collect_client.walkers+self.collect_client.vehicles:
                instance_token = self.dataset.update_instance(*self.collect_client.get_instance(scene_id,instance))
                instances_token[instance.get_actor().id] = instance_token
                samples_annotation_token[instance.get_actor().id] = ""
            
            for sensor in self.collect_client.sensors:
                calibrated_sensor_token = self.dataset.update_calibrated_sensor(scene_token,*self.collect_client.get_calibrated_sensor(sensor))
                calibrated_sensors_token[sensor.name] = calibrated_sensor_token
                samples_data_token[sensor.name] = ""

            sample_token = ""
            lidar_data = None
            lidar_transform = None
            radar_data = None
            radar_transform = None
            for count in range(int(self.config["collect_time"]/self.collect_client.settings.fixed_delta_seconds)):
                print("count:",count)
                self.collect_client.tick()
                if (count+1)%int(self.config["keyframe_time"]/self.collect_client.settings.fixed_delta_seconds) == 0:
                    sample_token = self.dataset.update_sample(sample_token,scene_token,*self.collect_client.get_sample())
                    for sensor in self.collect_client.sensors:
                        if sensor.bp_name in ['sensor.camera.rgb','sensor.other.radar','sensor.lidar.ray_cast']:
                            for idx,sample_data in enumerate(sensor.get_data_list()):
                                ego_pose_token = self.dataset.update_ego_pose(scene_token,calibrated_sensors_token[sensor.name],*self.collect_client.get_ego_pose(sample_data))
                                is_key_frame = False
                                if idx == len(sensor.get_data_list())-1:
                                    is_key_frame = True
                                    if sensor.bp_name == "sensor.lidar.ray_cast":
                                        lidar_data = sample_data
                                        lidar_transform = sensor.get_transform()
                                    if sensor.bp_name == "sensor.other.radar":
                                        radar_data = sample_data
                                        radar_transform = sensor.get_transform()
                                samples_data_token[sensor.name] = self.dataset.update_sample_data(samples_data_token[sensor.name],calibrated_sensors_token[sensor.name],sample_token,ego_pose_token,is_key_frame,*self.collect_client.get_sample_data(sample_data))

                    for instance in self.collect_client.walkers+self.collect_client.vehicles:
                        if self.collect_client.get_visibility() > 0:
                            samples_annotation_token[instance.get_actor().id]  = self.dataset.update_sample_annotation(samples_annotation_token[instance.get_actor().id],sample_token,*self.collect_client.get_sample_annotation(scene_id,instance,lidar_data,lidar_transform,radar_data,radar_transform))
                    
                    for sensor in self.collect_client.sensors:
                        sensor.get_data_list().clear()
        finally:
            self.collect_client.destroy_scene()