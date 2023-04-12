import carla
class Actor:
    def __init__(self,world,bp_name,location,rotation,options=None,attach_to=None):
        self.bp_name = bp_name
        self.world = world
        self.blueprint = world.get_blueprint_library().find(bp_name)
        if options is not None:
            for key in options:
                self.blueprint.set_attribute(key, options[key])
        self.transform = carla.Transform(carla.Location(**location),carla.Rotation(**rotation))
        self.attach_to = attach_to
        self.actor = None

    def set_actor(self,id):
        self.actor = self.world.get_actor(id)

    def spawn_actor(self):
        self.actor = self.world.spawn_actor(self.blueprint,self.transform,self.attach_to)

    def get_actor(self):
        return self.actor

    def destroy(self):
        self.actor.destroy()