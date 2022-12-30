from actor import Actor
import carla
class Vehicle(Actor):
    def __init__(self,path,**args):
        super().__init__(**args)
        self.path=[carla.Location(**location) for location in path]
        
    
    