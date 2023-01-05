import os
from utils import load,dump,get_token
import carla
from sensor import parse_image,parse_lidar_data,parse_radar_data

def save_image(image,path):
    image.save_to_disk(path)

def save_lidar_data(lidar_data,path):
    points = parse_lidar_data(lidar_data)
    points.tofile(path)

def save_radar_data(radar_data,path):
    points = parse_radar_data(radar_data)
    points.tofile(path)

def save_data(data,path):
    if isinstance(data,carla.Image):
        return save_image(data,path)
    elif isinstance(data,carla.RadarMeasurement):
        return parse_radar_data(data,path)
    elif isinstance(data,carla.LidarMeasurement):
        return save_lidar_data(data,path)   

class Dataset:
    def __init__(self,root,version,load=True):
        self.root = root
        self.version = version
        self.json_dir = os.path.join(root,version)
        self.data = {
            "attribute":[],
            "calibrated_sensor":[],
            "category":[],
            "ego_pose":[],
            "instance":[],
            "log":[],
            "map":[],
            "sample":[],
            "sample_annotation":[],
            "sample_data":[],
            "scene":[],
            "sensor":[],
            "visibility":[],
        }
        if load:
            self.load()

    def load(self):
        for key in self.data:
            json_path = os.path.join(self.json_dir,key+".json")
            self.data[key] = load(json_path)

    def save(self):
        for key in self.data:
            json_path = os.path.join(self.json_dir,key+".json")
            dump(self.data[key],json_path)

    def get_item(self,key,token):
        for item in self.data[key]:
            if item["token"] == token:
                return item

    def add_map(self,name,category):
        map_item = {}
        map_item["category"] = category
        map_item["token"] = get_token("map",name)
        map_item["filename"] = os.path.join("maps",map_item["token"]+".png")
        map_item["log_tokens"] = []
        self.data["map"].append(map_item)
        return map_item["token"]

    def add_log(self,map_token,date,time,timezone,vehicle,location):
        log_item = {}
        log_item["logfile"] = vehicle+"-"+date+"-"+time+timezone
        log_item["token"] = get_token("log",map_token+log_item["logfile"])
        log_item["vehicle"] = vehicle
        log_item["date_captured"] = date
        log_item["location"] = location
        self.data["log"].append(log_item)
        map_item = self.get_item("map",map_token)
        map_item["log_tokens"].append(log_item["token"])
        return log_item["token"]

    def add_sensor(self,channel,modality):
        sensor_item = {}
        sensor_item["token"] = get_token("sensor",channel)
        sensor_item["channel"] = channel
        sensor_item["modality"] = modality
        self.data["sensor"].append(sensor_item)
        return sensor_item["token"]

    def add_calibrated_sensor(self,scene_token,channel,translation,rotation,intrinsic):
        calibrated_sensor_item = {}
        calibrated_sensor_item["token"] = get_token("calibrated_sensor",scene_token+channel)
        calibrated_sensor_item["translation"] = translation
        calibrated_sensor_item["rotation"] = rotation
        calibrated_sensor_item["intrinsic"] = intrinsic
        self.data["calibrated_sensor"].append(calibrated_sensor_item)
        return calibrated_sensor_item["token"]

    def add_scene(self,log_token,id,description):
        scene_item = {}
        scene_item["name"] = "scene-"+str(id)
        scene_item["token"] = get_token("scene",log_token+scene_item["name"])
        scene_item["description"] = description
        scene_item["log_token"] = log_token
        scene_item["nbr_samples"] = 0
        scene_item["first_sample_token"] = ""
        scene_item["last_sample_token"] = ""
        self.data["scene"].append(scene_item)
        return scene_item["token"]

    def add_sample(self,scene_token,timestamp,prev):
        sample_item = {}
        sample_item["token"] = get_token("sample",scene_token+str(timestamp))
        sample_item["timestamp"] = timestamp
        sample_item["prev"] = prev
        sample_item["next"] = ""
        scene_item  = self.get_item("scene",scene_token)
        if prev == "":
            scene_item["first_sample_token"] = sample_item["token"]
        else:
            self.get_item("sample",prev)["next"] = sample_item["token"]
        scene_item["last_sample_token"] = sample_item["token"]
        scene_item["nbr_samples"] += 1
        self.data["sample"].append(sample_item)
        return sample_item["token"]

    def add_sample_data(self,calibrated_sensor_token,sample_token,ego_pose_token,is_key_frame,height,width,prev):
        sample_data_item = {}
        sample_data_item["token"] = ego_pose_token
        sample_data_item["sample_token"] = sample_token
        sample_data_item["ego_pose_token"] = ego_pose_token
        sample_data_item["calibrated_sensor_token"] = calibrated_sensor_token
        sample_data_item["timestamp"] = self.get_item("ego_pose",ego_pose_token)["timestamp"]
        sensor = self.get_item("sensor",self.get_item("calibrated_sensor",calibrated_sensor_token)["sensor_token"])
        if sensor["modality"] == "camera":
            sample_data_item["fileformat"] = "jpg"
        elif sensor["modality"] == "radar" or sensor["modality"] == "lidar":
             sample_data_item["fileformat"] = "pcd"
        sample_data_item["is_key_frame"] = is_key_frame
        sample_data_item["height"] = height
        sample_data_item["width"] = width
        sample_data_item["prev"] = prev
        sample_data_item["next"] = ""
        if prev != "":
            self.get_item("sample_data",prev)["next"] = ego_pose_token
        self.data["sample_data"].append(sample_data_item)
        return sample_data_item["token"]

    def add_ego_pose(self,scene_token,timestamp,rotation,translation):
        ego_pose_item = {}
        ego_pose_item["token"] = get_token("ego_pose",scene_token+str(timestamp))
        ego_pose_item["timestamp"] = timestamp
        ego_pose_item["rotation"] = rotation
        ego_pose_item["translation"] = translation
        self.data["ego_pose"].append(ego_pose_item)
        return ego_pose_item["token"]

    def add_visibility(self,description,level):
        visibility_item = {}
        visibility_item["token"] = str(len(self.data["visibility"]))
        visibility_item["description"] = description
        visibility_item["level"] = level
        self.data["visibility"].append(visibility_item)
        return visibility_item["token"]

    def add_attribute(self,name,description):
        attribute_item = {}
        attribute_item["token"] = get_token("attribute",name)
        attribute_item["name"] = name
        attribute_item["description"] = description
        self.data["attribute"].append(attribute_item)
        return attribute_item["token"]

    def add_category(self,name,description):
        category_item = {}
        category_item["token"] = get_token("category",name)
        category_item["name"] = name
        category_item["description"] = description
        self.data["category"].append(category_item)
        return category_item["token"]

    def add_instance(self,category_token,id):
        instance_item = {}
        instance_item["token"] = get_token("instance",id)
        instance_item["category_token"] = category_token
        instance_item["nbr_annotations"] = 0
        instance_item["first_annotation_token"] = ""
        instance_item["last_annotation_token"] = ""
        self.data["instance"].append(instance_item)
        return instance_item["token"]

    def add_sample_annotation(self,sample_token,instance_token,visibility_token,
                            attribute_tokens,translation,rotation,
                            size,prev,num_lidar_pts,num_radar_pts):
        sample_annotation_item = {}
        sample_annotation_item["token"] = get_token("sample_annotation",sample_token+instance_token)
        sample_annotation_item["sample_token"] = sample_token
        sample_annotation_item["instance_token"] = instance_token
        sample_annotation_item["visibility_token"] = visibility_token
        sample_annotation_item["attribute_tokens"] = attribute_tokens
        sample_annotation_item["translation"] = translation
        sample_annotation_item["rotation"] = rotation
        sample_annotation_item["size"] = size
        sample_annotation_item["prev"] = prev
        sample_annotation_item["next"] = ""
        sample_annotation_item["num_lidar_pts"] = num_lidar_pts
        sample_annotation_item["num_radar_pts"] = num_radar_pts
        instance_item  = self.get_item("instance",instance_token)
        if prev == "":
            instance_item["first_annotation_token"] = instance_item["token"]
        else:
            self.get_item("instance",prev)["next"] = instance_item["token"]
        instance_item["last_annotation_token"] = instance_item["token"]
        instance_item["nbr_annotations"] += 1
        self.data["sample_annotation"].append(sample_annotation_item)
        return sample_annotation_item["token"]

    def save_data(self,sample_data_token,data):
        sample_data_item = self.get_item("sample_data",sample_data_token)
        channel = self.get_item("sensor",self.get_item("calibrated_sensor",sample_data_item["calibrated_sensor_token"])["sensor_token"])["channel"]
        if sample_data_item["is_key_frame"]:
            dir = "samples"
        else:
            dir = "sweeps"
        log_file = self.get_item("log",self.get_item("scene",self.get_item("sample",sample_data_item["sample_token"])["scene_token"])["log_token"])["log_file"]
        filename = log_file+"_"+channel+"_"+sample_data_item["timestamp"]+"."+sample_data_item["fileformat"]
        path = os.path.join(self.root,dir,channel,filename)
        save_data(path,data)