from .client import Client
from .dataset import Dataset
import traceback

class Generator:
    def __init__(self,config):
        self.config = config
        self.collect_client = Client(self.config["client"])

    def generate_dataset(self,load=False):
        self.dataset = Dataset(**self.config["dataset"],load=load)
        print(self.dataset.data["progress"])
        self.dataset.save()
        for sensor in self.config["sensors"]:
            self.dataset.update_sensor(sensor["name"],sensor["modality"])
        for category in self.config["categories"]:
            self.dataset.update_category(category["name"],category["description"])
        for attribute in self.config["attributes"]:
            self.dataset.update_attribute(attribute["name"],category["description"])
        for visibility in self.config["visibility"]:
            self.dataset.update_visibility(visibility["description"],visibility["level"])

        for world_idx in range(self.dataset.data["progress"]["current_world_index"],len(self.config["worlds"])):
            world_config = self.config["worlds"][world_idx]
            self.dataset.update_world_index(world_idx)
            try:
                self.collect_client.generate_world(world_config)
                map_token = self.dataset.update_map(world_config["map_name"],world_config["map_category"])
                for capture_idx in range(self.dataset.data["progress"]["current_capture_index"],len(world_config["captures"])):
                    capture_config = world_config["captures"][capture_idx]
                    self.dataset.update_capture_index(world_idx)
                    log_token = self.dataset.update_log(map_token,capture_config["date"],capture_config["time"],
                                            capture_config["timezone"],capture_config["capture_vehicle"],capture_config["location"])
                    for scene_idx in range(self.dataset.data["progress"]["current_scene_index"],len(capture_config["scenes"])):
                        scene_config = capture_config["scenes"][scene_idx]
                        self.dataset.update_scene_index(scene_idx)
                        for scene_count in range(self.dataset.data["progress"]["current_scene_index"],scene_config["count"]):
                            self.dataset.update_scene_count(scene_count)
                            scene_token = self.add_one_scene(log_token,scene_idx,scene_config)
                            self.dataset.save()
            except:
                traceback.print_exc()
            finally:
                self.collect_client.destroy_world()

    def add_one_scene(self,log_token,scene_id,scene_config):
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
            for frame_count in range(int(scene_config["collect_time"]/self.collect_client.settings.fixed_delta_seconds)):
                print("frame count:",frame_count)
                self.collect_client.tick()
                if (frame_count+1)%int(scene_config["keyframe_time"]/self.collect_client.settings.fixed_delta_seconds) == 0:
                    sample_token = self.dataset.update_sample(sample_token,scene_token,*self.collect_client.get_sample())
                    for sensor in self.collect_client.sensors:
                        if sensor.bp_name in ['sensor.camera.rgb','sensor.other.radar','sensor.lidar.ray_cast']:
                            for idx,sample_data in enumerate(sensor.get_data_list()):
                                ego_pose_token = self.dataset.update_ego_pose(scene_token,calibrated_sensors_token[sensor.name],*self.collect_client.get_ego_pose(sample_data))
                                is_key_frame = False
                                if idx == len(sensor.get_data_list())-1:
                                    is_key_frame = True
                                samples_data_token[sensor.name] = self.dataset.update_sample_data(samples_data_token[sensor.name],calibrated_sensors_token[sensor.name],sample_token,ego_pose_token,is_key_frame,*self.collect_client.get_sample_data(sample_data))

                    for instance in self.collect_client.walkers+self.collect_client.vehicles:
                        if self.collect_client.get_visibility(instance) > 0:
                            samples_annotation_token[instance.get_actor().id]  = self.dataset.update_sample_annotation(samples_annotation_token[instance.get_actor().id],sample_token,*self.collect_client.get_sample_annotation(scene_id,instance))
                    
                    for sensor in self.collect_client.sensors:
                        sensor.get_data_list().clear()
        finally:
            self.collect_client.destroy_scene()