from collect_client import CollectClient
from dataset import Dataset
import yaml

config_path = "./config.yaml"
dataset = Dataset("./","carla_1.0")
map_token = dataset.add_map("Town01")
log_token = dataset.add_log(map_token,xxxxx)
with open(config_path,'r') as f:
    configs = yaml.load(f.read(),Loader=yaml.CLoader)
for config in configs:
    client = CollectClient("./config.yaml",dataset)
    scene_token = dataset.add_scene(log_token,xxxxx)
    client.run(scene_token)
dataset.save()