import numpy as np
from actor import Actor

def parse_image(image):
    array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
    image = array.copy()
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

def save_image(image,path):
    image.save_to_disk(path)

def save_lidar_data(lidar_data,path):
    points = parse_lidar_data(lidar_data)
    points.tofile(path)

def save_radar_data(radar_data,path):
    points = parse_radar_data(radar_data)
    points.tofile(path)

class Sensor(Actor):
    def __init__(self, name, **args):
        super().__init__(**args)
        self.name = name
        self.data_list = []
    
    def get_data(self):
        return self.data_list
    
    def set_actor(self, id):
        super().set_actor(id)
        self.actor.listen(self.add_data)

    def get_last_data(self):
        return self.data_list[-1]

    def add_data(self,data):
        self.data_list.append((self.actor.parent.get_location(),data))


        
            

