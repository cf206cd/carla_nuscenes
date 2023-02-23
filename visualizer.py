import carla
from client import Client
from sensor import Sensor,parse_image
import pygame
import yaml
from yamlinclude import YamlIncludeConstructor
import traceback
from PIL import Image
YamlIncludeConstructor.add_to_loader_class(loader_class=yaml.FullLoader)
config_path = "./configs/vis_config.yaml"
with open(config_path,'r') as f:
    config = yaml.load(f.read(),Loader=yaml.FullLoader)

class KeyboardControl:
    def __init__(self,player):
        self.player = player
        self.steer=0
        self.throttle=0
        self.brake=0
        self.reverse = False

    def parse_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    return True
        #         if event.key== pygame.K_BACKSPACE:
        #             self.reverse = not self.reverse

        # keys_pressed = pygame.key.get_pressed()
        # if keys_pressed[pygame.K_LEFT]:
        #     self.steer=max(-0.7,self.steer-0.01)
        # if keys_pressed[pygame.K_RIGHT]:
        #     self.steer=min(0.7,self.steer+0.01)
        # if keys_pressed[pygame.K_UP]:
        #     if self.throttle == 0:
        #         self.throttle = 0.4
        #     self.throttle=min(1,self.throttle+0.01)
        # else:
        #     self.throttle=0
        # if keys_pressed[pygame.K_DOWN]:
        #     if self.brake == 0:
        #         self.brake = 0.4
        #     self.brake=min(1,self.brake+0.01)
        # else:
        #     self.brake=0
        # control = carla.VehicleControl(self.throttle,self.steer,self.brake,reverse = self.reverse)
        # print(control)
        # self.player.apply_control(control)
        # return False
class Visualizer:
    def __init__(self,config):
        self.config = config
        self.client = Client(self.config["client"])

    def run(self):
        try:
            self.client.generate_world(self.config["world"])
            self.client.generate_random_scene(self.config["scene"])
            pygame.init()
            self.display = pygame.display.set_mode((self.config["ui"]["width"],self.config["ui"]["height"]),pygame.HWSURFACE | pygame.DOUBLEBUF)
            self.display.fill((0,0,0))
            pygame.display.flip()
            self.client.ego_vehicle.get_actor().set_autopilot(True)#also can set False
            self.keyboard_control = KeyboardControl(self.client.ego_vehicle.get_actor())
            clock=pygame.time.Clock()
            self.vis_camera = Sensor(world=self.client.world, attach_to=self.client.ego_vehicle.get_actor(), **self.config["vis_camera"])
            self.vis_camera.spawn_actor()
            self.client.tick()
            w=self.config["ui"]["width"]
            h=self.config["ui"]["height"]
            rect = {
                "FRAME":(0,0,w*3//4,h*3//5),
                "CAM_FRONT_LEFT":(0,h*3//5,w//4,h//5),
                "CAM_FRONT":(w//4,h*3//5,w//4,h//5),
                "CAM_FRONT_RIGHT":(w*2//4,h*3//5,w//4,h//5),
                "CAM_BACK_LEFT":(0,h*4//5,w//4,h//5),
                "CAM_BACK":(w//4,h*4//5,w//4,h//5),
                "CAM_BACK_RIGHT":(w*2//4,h*4//5,w//4,h//5),
                "GROUND_TRUTH":(w*3//4,0,w//4,h//2),
                "PREDICTION":(w*3//4,h//2,w//4,h//2)
            }
            import time
            while True:
                clock.tick()
                if self.keyboard_control.parse_events():
                    return
                else:
                    self.client.tick()
                    if self.vis_camera.get_last_data():
                        image = self.vis_camera.get_last_data()[1]
                        self.display_image(image,rect["FRAME"])
                        self.vis_camera.get_data_list().clear()
                    for sensor in self.client.sensors:
                        if sensor.name in rect.keys() and sensor.get_data_list():
                            image = sensor.get_last_data()[1]
                            self.display_image(image,rect[sensor.name])
                            sensor.get_data_list().clear()
                    pygame.display.flip()
                    
        except:
            traceback.print_exc()
        finally:
            self.client.destroy_scene()
            self.client.destroy_world()

    def display_image(self,image,rect):
        b,g,r,a = Image.frombuffer("RGBA",(image.width,image.height),image.raw_data).split()
        frame = Image.merge("RGBA", (r,g,b,a)).resize(rect[2:])
        mode = frame.mode
        size = frame.size
        data = frame.tobytes()
        py_image = pygame.image.frombytes(data, size, mode)
        self.display.blit(py_image,rect[0:2])
    
    def get_input(self,):
        pass
    
    def predict(self,):
        pass
vis  = Visualizer(config)
vis.run()