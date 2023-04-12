from .actor import Actor
import carla
class Vehicle(Actor):
    def __init__(self,path=[],**args):
        super().__init__(**args)
        self.path=[carla.Location(**location) for location in path]
        
    def get_transform(self):
        location = self.actor.get_transform().transform(self.actor.bounding_box.location)
        rotation = self.actor.get_transform().rotation
        return carla.Transform(location,rotation)

    def get_bbox(self):
        return self.actor.bounding_box.get_world_vertices(self.actor.get_transform())

    def get_size(self):
        return self.actor.bounding_box.extent*2