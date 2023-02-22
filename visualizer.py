import carla
from carla_client import CarlaClient
from sensor import Sensor,parse_image
import pygame
import yaml
from yamlinclude import YamlIncludeConstructor
from dataset import save_data

YamlIncludeConstructor.add_to_loader_class(loader_class=yaml.FullLoader)
config_path = "./configs/viz_config.yaml"
with open(config_path,'r') as f:
    config = yaml.load(f.read(),Loader=yaml.FullLoader)

class KeyboardControl:
    def __init__(self,player):
        self.player = player
        self.steer=0
        self.throttle=0

    def parse_events(self):
        for event in pygame.event.get():
            print("FUCK")
            if event.type == pygame.QUIT:
                return True
            elif event.type == pygame.KEYDOWN:
                print("DOWN")
                if event.key == pygame.K_LEFT:
                    self.steer=max(0,self.steer-0.1)
                if event.key == pygame.K_RIGHT:
                    self.steer=min(1,self.steer+0.1)
                if event.key == pygame.K_UP:
                    self.throttle=min(1,self.throttle+0.1)
                if event.key == pygame.K_DOWN:
                    self.throttle=max(0,self.throttle-0.1)
                if event.key == pygame.K_ESCAPE:
                    return True
            control = carla.VehicleControl(self.throttle,self.steer)
            print(control)
            self.player.apply_control(control)
            return False
class Visualizer:
    def __init__(self,config):
        self.config = config
        self.client = CarlaClient(self.config["client"])

    def run(self):
        self.client.generate_world(self.config["world"])
        self.client.generate_random_scene(self.config["scene"])
        pygame.init()
        display = pygame.display.set_mode((self.config["ui"]["width"],self.config["ui"]["height"]),pygame.HWSURFACE | pygame.DOUBLEBUF)
        display.fill((0,0,0))
        pygame.display.flip()
        self.client.ego_vehicle.get_actor().set_autopilot(False)
        self.keyboard_control = KeyboardControl(self.client.ego_vehicle.get_actor())
        clock=pygame.time.Clock()
        self.viz_camera = Sensor(world=self.client.world, attach_to=self.client.ego_vehicle.get_actor(), **self.config["viz_camera"])
        self.viz_camera.spawn_actor()
        self.client.tick()
        import time
        while True:
            clock.tick_busy_loop(60)
            if self.keyboard_control.parse_events():
                return
            else:
                t1 = time.time()
                self.client.tick()
                t2 = time.time()
                if self.viz_camera.get_data_list():
                    #save_data(self.viz_camera.get_last_data()[1],"./q.png")
                    array = parse_image(self.viz_camera.get_last_data()[1])
                    array = array[:, :, :3]
                    array = array[:, :, ::-1]
                    array = array.swapaxes(0, 1)
                    self.surface = pygame.surfarray.make_surface(array)
                    t3 = time.time()
                    display.blit(self.surface,(0,0))
                    t4 = time.time()
                    pygame.display.flip()
                    t5 = time.time()
                    self.viz_camera.get_data_list().clear()
                t6 = time.time()
                print(t2-t1,t3-t2,t4-t3,t5-t4,t6-t5)
vis  = Visualizer(config)
vis.run()