import carla
from collect_client import CollectClient
import pygame
import yaml
from yamlinclude import YamlIncludeConstructor
YamlIncludeConstructor.add_to_loader_class(loader_class=yaml.FullLoader)
config_path = "./configs/config.yaml"
with open(config_path,'r') as f:
    config = yaml.load(f.read(),Loader=yaml.FullLoader)
class Visualizer:
    def __init__(self,config):
        self.config = config
        self.collect_client = CollectClient(self.config["client"])

    def run(self):
        for world_config in self.config["worlds"]:
            self.collect_client.generate_world(world_config)
            for capture_config in world_config["captures"]:
                for scene_config in capture_config["scenes"]:
                    self.collect_client.generate_random_scene(scene_config)
                    while True:
                        self.collect_client.tick()

vis  = Visualizer(config)
vis.run()