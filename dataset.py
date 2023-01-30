import os
from utils import load,dump,generate_token
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
        save_image(data,path)
    elif isinstance(data,carla.RadarMeasurement):
        save_radar_data(data,path)
    elif isinstance(data,carla.LidarMeasurement):
        save_lidar_data(data,path)   

def mkdir(path):
    if not os.path.exists(path):
        os.mkdir(path)

class Dataset:
    def __init__(self,root,version,load=False):
        self.root = root
        self.version = version
        self.json_dir = os.path.join(root,version)
        mkdir(root)
        mkdir(self.json_dir)
        mkdir(os.path.join(root,"maps"))
        mkdir(os.path.join(root,"samples"))
        mkdir(os.path.join(root,"sweeps"))

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
            "current_scene_count":0
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
        return None

    def update_map(self,name,category,replace=True):
        map_item = {}
        map_item["category"] = category
        map_item["token"] = generate_token("map",name)
        map_item["filename"] = os.path.join("maps",map_item["token"]+".png")
        map_item["log_tokens"] = []
        if self.get_item("map",map_item["token"]) is None:
            self.data["map"].append(map_item)
        elif replace:
            self.data["map"].remove(self.get_item("map",map_item["token"]))
            self.data["map"].append(map_item)
        return map_item["token"]

    def update_log(self,map_token,date,time,timezone,vehicle,location,replace=True):
        log_item = {}
        log_item["logfile"] = vehicle+"-"+date+"-"+time+timezone
        log_item["token"] = generate_token("log",map_token+log_item["logfile"])
        log_item["vehicle"] = vehicle
        log_item["date_captured"] = date
        log_item["location"] = location
        map_item = self.get_item("map",map_token)
        map_item["log_tokens"].append(log_item["token"])
        if self.get_item("log",log_item["token"]) is None:
            self.data["log"].append(log_item)
        elif replace:
            self.data["log"].remove(self.get_item("log",log_item["token"]))
            self.data["log"].append(log_item)
        return log_item["token"]

    def update_sensor(self,channel,modality,replace=True):
        sensor_item = {}
        sensor_item["token"] = generate_token("sensor",channel)
        sensor_item["channel"] = channel
        sensor_item["modality"] = modality
        if self.get_item("sensor",sensor_item["token"]) is None:
            self.data["sensor"].append(sensor_item)
        elif replace:
            self.data["sensor"].remove(self.get_item("sensor",sensor_item["token"]))
            self.data["sensor"].append(sensor_item)
        mkdir(os.path.join(self.root,"samples",channel))
        mkdir(os.path.join(self.root,"sweeps",channel))
        return sensor_item["token"]

    def update_calibrated_sensor(self,scene_token,sensor_token,channel,translation,rotation,intrinsic,replace=True):
        calibrated_sensor_item = {}
        calibrated_sensor_item["token"] = generate_token("calibrated_sensor",scene_token+channel)
        calibrated_sensor_item["sensor_token"] = sensor_token
        calibrated_sensor_item["translation"] = translation
        calibrated_sensor_item["rotation"] = rotation
        calibrated_sensor_item["camera_intrinsic"] = intrinsic
        if self.get_item("calibrated_sensor",calibrated_sensor_item["token"]) is None:
            self.data["calibrated_sensor"].append(calibrated_sensor_item)
        elif replace:
            self.data["calibrated_sensor"].remove(self.get_item("calibrated_sensor",calibrated_sensor_item["token"]))
            self.data["calibrated_sensor"].append(calibrated_sensor_item)
        return calibrated_sensor_item["token"]

    def update_scene(self,log_token,id,description,replace=True):
        scene_item = {}
        scene_item["name"] = "scene-"+str(id)
        scene_item["token"] = generate_token("scene",log_token+scene_item["name"])
        scene_item["description"] = description
        scene_item["log_token"] = log_token
        scene_item["nbr_samples"] = 0
        scene_item["first_sample_token"] = ""
        scene_item["last_sample_token"] = ""
        if self.get_item("scene",scene_item["token"]) is None:
            self.data["scene"].append(scene_item)
        elif replace:
            self.data["scene"].remove(self.get_item("scene",scene_item["token"]))
            self.data["scene"].append(scene_item)
        return scene_item["token"]

    def update_sample(self,prev,scene_token,timestamp,replace=True):
        sample_item = {}
        sample_item["token"] = generate_token("sample",scene_token+str(timestamp))
        sample_item["timestamp"] = timestamp
        sample_item["prev"] = prev
        sample_item["next"] = ""
        sample_item["scene_token"] = scene_token
        scene_item  = self.get_item("scene",scene_token)
        if prev == "":
            scene_item["first_sample_token"] = sample_item["token"]
        else:
            self.get_item("sample",prev)["next"] = sample_item["token"]
        scene_item["last_sample_token"] = sample_item["token"]
        scene_item["nbr_samples"] += 1
        if self.get_item("sample",sample_item["token"]) is None:
            self.data["sample"].append(sample_item)
        elif replace:
            self.data["sample"].remove(self.get_item("sample",sample_item["token"]))
            self.data["sample"].append(sample_item)
        return sample_item["token"]

    def update_sample_data(self,prev,calibrated_sensor_token,sample_token,ego_pose_token,is_key_frame,sample_data,height,width,replace=True):
        sample_data_item = {}
        sample_data_item["token"] = ego_pose_token
        sample_data_item["sample_token"] = sample_token
        sample_data_item["ego_pose_token"] = ego_pose_token
        sample_data_item["calibrated_sensor_token"] = calibrated_sensor_token
        sample_data_item["timestamp"] = self.get_item("ego_pose",ego_pose_token)["timestamp"]
        sensor = self.get_item("sensor",self.get_item("calibrated_sensor",calibrated_sensor_token)["sensor_token"])
        if sensor["modality"] == "camera":
            sample_data_item["fileformat"] = "jpg"
        elif sensor["modality"] == "radar":
            sample_data_item["fileformat"] = "pcd"
        elif sensor["modality"] == "lidar":
            sample_data_item["fileformat"] = "pcd.bin"
        sample_data_item["is_key_frame"] = is_key_frame
        sample_data_item["height"] = height
        sample_data_item["width"] = width
        sample_data_item["prev"] = prev
        sample_data_item["next"] = ""
        filename = self.get_filename(sample_data_item)
        print("save file:",filename)
        save_data(sample_data[1],os.path.join(self.root,filename))
        sample_data_item["filename"] = filename
        if prev != "":
            self.get_item("sample_data",prev)["next"] = ego_pose_token
        if self.get_item("sample_data",sample_data_item["token"]) is None:
            self.data["sample_data"].append(sample_data_item)
        elif replace:
            self.data["sample_data"].remove(self.get_item("sample_data",sample_data_item["token"]))
            self.data["sample_data"].append(sample_data_item)
        return sample_data_item["token"]

    def update_ego_pose(self,scene_token,calibrated_sensor_token,timestamp,translation,rotation,replace=True):
        ego_pose_item = {}
        ego_pose_item["token"] = generate_token("ego_pose",scene_token+calibrated_sensor_token+str(timestamp))
        ego_pose_item["timestamp"] = timestamp
        ego_pose_item["rotation"] = rotation
        ego_pose_item["translation"] = translation
        if self.get_item("ego_pose",ego_pose_item["token"]) is None:
            self.data["ego_pose"].append(ego_pose_item)
        elif replace:
            self.data["ego_pose"].remove(self.get_item("ego_pose",ego_pose_item["token"]))
            self.data["ego_pose"].append(ego_pose_item)
        return ego_pose_item["token"]

    def update_visibility(self,description,level,replace=True):
        visibility_item = {}
        visibility_item["token"] = str(len(self.data["visibility"]))
        visibility_item["description"] = description
        visibility_item["level"] = level
        if self.get_item("visibility",visibility_item["token"]) is None:
            self.data["visibility"].append(visibility_item)
        elif replace:
            self.data["visibility"].remove(self.get_item("visibility",visibility_item["token"]))
            self.data["visibility"].append(visibility_item)
        return visibility_item["token"]

    def update_attribute(self,name,description,replace=True):
        attribute_item = {}
        attribute_item["token"] = generate_token("attribute",name)
        attribute_item["name"] = name
        attribute_item["description"] = description
        if self.get_item("attribute",attribute_item["token"]) is None:
            self.data["attribute"].append(attribute_item)
        elif replace:
            self.data["attribute"].remove(self.get_item("attribute",attribute_item["token"]))
            self.data["attribute"].append(attribute_item)
        return attribute_item["token"]

    def update_category(self,name,description,replace=True):
        category_item = {}
        category_item["token"] = generate_token("category",name)
        category_item["name"] = name
        category_item["description"] = description
        if self.get_item("category",category_item["token"]) is None:
            self.data["category"].append(category_item)
        elif replace:
            self.data["category"].remove(self.get_item("category",category_item["token"]))
            self.data["category"].append(category_item)
        return category_item["token"]

    def update_instance(self,category_token,id,replace=True):
        instance_item = {}
        instance_item["token"] = generate_token("instance",id)
        instance_item["category_token"] = category_token
        instance_item["nbr_annotations"] = 0
        instance_item["first_annotation_token"] = ""
        instance_item["last_annotation_token"] = ""
        if self.get_item("instance",instance_item["token"]) is None:
            self.data["instance"].append(instance_item)
        elif replace:
            self.data["instance"].remove(self.get_item("instance",instance_item["token"]))
            self.data["instance"].append(instance_item)
        return instance_item["token"]

    def update_sample_annotation(self,prev,sample_token,instance_token,visibility_token,
                            attribute_tokens,translation,rotation,
                            size,num_lidar_pts,num_radar_pts,replace=True):
        sample_annotation_item = {}
        sample_annotation_item["token"] = generate_token("sample_annotation",sample_token+instance_token)
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
            instance_item["first_annotation_token"] = sample_annotation_item["token"]
        else:
            self.get_item("sample_annotation",prev)["next"] = sample_annotation_item["token"]
        instance_item["last_annotation_token"] = sample_annotation_item["token"]
        instance_item["nbr_annotations"] += 1
        if self.get_item("sample_annotation",sample_annotation_item["token"]) is None:
            self.data["sample_annotation"].append(sample_annotation_item)
        elif replace:
            self.data["sample_annotation"].remove(self.get_item("sample_annotation",sample_annotation_item["token"]))
            self.data["sample_annotation"].append(sample_annotation_item)
        return sample_annotation_item["token"]

    def get_filename(self,sample_data_item):
        channel = self.get_item("sensor",self.get_item("calibrated_sensor",sample_data_item["calibrated_sensor_token"])["sensor_token"])["channel"]
        if sample_data_item["is_key_frame"]:
            dir = "samples"
        else:
            dir = "sweeps"
        log_file = self.get_item("log",self.get_item("scene",self.get_item("sample",sample_data_item["sample_token"])["scene_token"])["log_token"])["logfile"]
        name = log_file+"_"+channel+"_"+str(sample_data_item["timestamp"])+"."+sample_data_item["fileformat"]
        filename = os.path.join(dir,channel,name)
        return filename

    def update_scene_count(self):
        self.data["current_scene_count"]+=1