from runner import Runner
import os
import yaml
from yamlinclude import YamlIncludeConstructor
YamlIncludeConstructor.add_to_loader_class(loader_class=yaml.FullLoader)
config_path = "./configs/random_config.yaml"
with open(config_path,'r') as f:
    config = yaml.load(f.read(),Loader=yaml.FullLoader)
runner = Runner(config)
if os.path.exists(config["dataset"]["root"]):
    runner.continue_generating_random_dataset()
else:
    runner.generate_new_random_dataset()