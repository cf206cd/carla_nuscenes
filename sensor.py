import numpy as np
import carla
from actor import Actor

def parse_image(image):
    array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
    array = array.copy()
    array = np.reshape(array, (image.height, image.width, 4))
    return array

def parse_lidar_data(lidar_data):
    points = np.frombuffer(lidar_data.raw_data, dtype=np.dtype('f4'))
    points = points.copy()
    points = np.reshape(points, (int(points.shape[0] / 4), 4))
    tmp = np.zeros((points.shape[0], 1))
    points = np.append(points, tmp, axis = 1)
    return points

def parse_radar_data(radar_data):
    points = np.frombuffer(radar_data.raw_data, dtype=np.dtype('f4'))
    points = points.copy()
    return points

def parse_data(data):
    if isinstance(data,carla.Image):
        return parse_image(data)
    elif isinstance(data,carla.RadarMeasurement):
        return parse_radar_data(data)
    elif isinstance(data,carla.LidarMeasurement):
        return parse_lidar_data(data)

def get_data_shape(data):
    if isinstance(data,carla.Image):
        return data.height,data.width
    else:
        return 0,0
class Sensor(Actor):
    def __init__(self, name, **args):
        super().__init__(**args)
        self.name = name
        self.data_list = []
    
    def get_data_list(self):
        return self.data_list
    
    def set_actor(self, id):
        super().set_actor(id)
        self.actor.listen(self.add_data)

    def get_last_data(self):
        return self.data_list

    def add_data(self,data):
        self.data_list.append((self.actor.parent.get_transform(),data))
