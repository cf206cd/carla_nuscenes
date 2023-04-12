from .actor import Actor
import carla
class Walker(Actor):
    def __init__(self,destination=None,**args):
        super().__init__(**args)
        if self.blueprint.has_attribute('is_invincible'):
            self.blueprint.set_attribute('is_invincible', 'false')

        if destination is None:
            self.destination = self.world.get_random_location_from_navigation()
        else:
            self.destination = carla.Location(**destination)

        self.controller = None
    
    def set_controller(self,id):
        self.controller = self.world.get_actor(id)

    def start(self):
        self.controller.start()
        self.controller.go_to_location(self.destination)

    def stop(self):
        self.controller.stop()

    def get_transform(self):
        location = self.actor.get_transform().transform(self.actor.bounding_box.location)
        rotation = self.actor.get_transform().rotation
        return carla.Transform(location,rotation)
    
    def get_bbox(self):
        return self.actor.bounding_box.get_world_vertices(self.actor.get_transform())

    def get_size(self):
        return self.actor.bounding_box.extent*2