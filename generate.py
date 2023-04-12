from carla_nuscenes.generator import Generator
import os
import yaml
from yamlinclude import YamlIncludeConstructor
YamlIncludeConstructor.add_to_loader_class(loader_class=yaml.FullLoader)
config_path = "./configs/config.yaml"
with open(config_path,'r') as f:
    config = yaml.load(f.read(),Loader=yaml.FullLoader)
runner = Generator(config)
if os.path.exists(config["dataset"]["root"]):
    runner.generate_dataset(True)
else:
    runner.generate_dataset(False)