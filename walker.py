from actor import Actor
import carla
class Walker(Actor):
    def __init__(self,destination,**args):
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