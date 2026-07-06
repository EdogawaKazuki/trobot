from abc import ABC, abstractmethod

class MotorStatus:
    def __init__(self, position=0.0, rpm=0.0, current=0.0, temperature=0.0):
        self.position = position
        self.rpm = rpm
        self.current = current
        self.temperature = temperature

class MotorConfig:
    def __init__(self, **kwargs):
        for key, value in kwargs.items():
            setattr(self, key, value)

class Motor(ABC):
    def __init__(self, bus=None, can_id=None, offset=0, max_current=10):
        self.bus = bus
        self.can_id = can_id
        self.offset = offset
        self.revolutions = 0
        self.max_current = max_current
        self.status = None
        self.config = MotorConfig()
    
    def getMotorConfig(self):
        pass

    def setMotorConfig(self, config):
        pass

    def getMotorStatus(self):
        pass

    def setCurrent(self, current):
        pass

    def setPosition(self, position):
        pass