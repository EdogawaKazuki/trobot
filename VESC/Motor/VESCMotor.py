from asyncio.log import logger
from Motor.Motor import Motor, MotorConfig, MotorStatus
from pyvesc.VESC.messages.setters import SetDutyCycle, SetCurrent, SetRPM
from pyvesc.VESC.messages.getters import GetValues, GetRotorPosition, GetIMUData, GetMcConf
import asyncio
import logging

class VESCMotor(Motor):
    def __init__(self, bus=None, can_id=None, offset=0, max_current=10, radius=0.026, gear_ratio=8):
        super().__init__(bus, can_id, offset, max_current)
        self.bus = bus
        self.init = False
        self.radius = radius
        self.gear_ratio = gear_ratio
    async def getMotorConfig(self):
        # Implementation specific to VESC motor
        await self.bus.get_motor_config(can_id=self.can_id)
        while not self.init:
            await asyncio.sleep(0.1)
        return self.config

    def setMotorConfig(self, config):
        # Implementation specific to VESC motor
        self.config = config
        self.init = True

    def getMotorStatus(self):
        # Implementation specific to VESC motor
        return self.status
    
    def setMotorStatus(self, status):
        if self.status is not None and abs(self.status.pid_pos_now - status.pid_pos_now) > 180:
            if self.status.pid_pos_now > status.pid_pos_now:
                self.revolutions += 1
            else:
                self.revolutions -= 1

        self.status = status
        abs_pos = self.status.pid_pos_now + self.revolutions * 360
        setattr(self.status, 'absolute_position', abs_pos)

    def setForceLimit(self, max_force):
        max_current = self.computeForce2Current(max_force)
        self.setCurrentLimit(max_current)

    def setCurrentLimit(self, max_current):
        self.max_current = max_current
        print(f"Set max current to {self.max_current}A")

    def computeForce2Current(self, force):
        return (force * self.radius) / self.gear_ratio / (3 / 2 * self.config.foc_motor_flux_linkage * 14)
    
    async def setForce(self, force):
        current = self.computeForce2Current(force)
        await self.setCurrent(current)

    async def setCurrent(self, current):
        # Implementation specific to VESC motor
        if abs(current) > self.max_current:
            logging.error("Current exceeds maximum limit")
        await self.bus.set_current(current, can_id=self.can_id)

    async def setPosition(self, position):
        # Implementation specific to VESC motor
        await self.bus.set_position(position, can_id=self.can_id)
    
    async def requestStatusUpdate(self):
        # print(f"Requesting status update for CAN ID {self.can_id}")
        await self.bus.get_values_selective(can_id=self.can_id, mask=0x30084)