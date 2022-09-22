from .CameraPedestal import CameraPedestal
from .Manipulator import Manipulator

class CelestialOceanAPI:

    def __init__(self, master):
        self.master = master
        self.cameraPedestal = CameraPedestal(self.master)
        self.manipulator = Manipulator(self.master)
