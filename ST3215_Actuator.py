import os
import time
from python_st3215 import ST3215
import numpy as np
from enum import Enum

class Info(Enum):
    POSITION = 0
    POS = 0
    VELOCITY = 1
    VEL = 1
    LOAD = 2
    TEMPERATURE = 3
    TEMP = 3
    VOLTAGE = 4
    VOLT = 4
    CURRENT = 5
    CURR = 5
    MAX_INFO = 6

class MotorMode(Enum):
    POS = 0
    SPEED = 1
    PWM = 2
    STEP = 3
    IDLE = 4

class Actuator:
    def __init__(self, port, ids=[1,]):
        self.port = port
        self.ids = ids
        self.numActuator = len(ids)
        self.controller = ST3215(port)
        self.servo = [None] * self.numActuator
        # self.pos_zero = [0] * self.numActuator
        self.pos_zero = np.zeros(self.numActuator)

        self.__active = True

        for k, idty in enumerate(self.ids):
            self.servo[k] = self.controller.wrap_servo(idty)
        self.indices = list(range(self.numActuator))
        self.direction = np.full(self.numActuator, 1)

        # Status
        # self.mode = 

        # Attribute
        self.pos_res = 360/4096

        self.torque_disable()
    
    def __del__(self):
        # cleanup
        self.cleanup()

    def cleanup(self):
        if (self.__active):
            self.torque_disable()
            self.controller.close()
            self.__active = False

    def __no_indices(self, indices=None):
        if (indices is None):
            indices = list(range(self.numActuator))
        return indices
    
    def torque_enable(self, indices=None):
        indices = self.__no_indices(indices)
        for k, i in enumerate(indices):
            self.servo[k].sram.torque_enable()

    def torque_disable(self, indices=None):
        indices = self.__no_indices(indices)
        for k, i in enumerate(indices):
            self.servo[k].sram.torque_disable()
    

    def get_raw_step(self, indices=None):
        indices = self.__no_indices(indices)
        return (self.__get_info(Info.POS, indices))
    
    def get_pos(self, indices=None):
        indices = self.__no_indices(indices)
        # return (self.pos_zero[indices] - self.__get_info("pos", indices)) * self.pos_res
        return (self.__get_info(Info.POS, indices) - self.pos_zero[indices]) * self.direction[indices] * self.pos_res

    def get_vel(self, indices=None):
        return self.__get_info(Info.VEL, indices) * self.pos_res
    
    def get_load(self, indices=None):
        return self.__get_info(Info.LOAD, indices) * .1
    
    def get_temp(self, indices=None):
        return self.__get_info(Info.TEMP, indices)
    
    def get_volt(self, indices=None):
        return self.__get_info(Info.VOLT, indices) * .1
    
    def get_curr(self, indices=None):
        return self.__get_info(Info.CURR, indices) * 6.5e-3

    def __get_info(self, info_type, indices=None):
        if indices is None:
            indices = list(range(self.numActuator))
            
        readers = {
            Info.POS:   lambda i: self.servo[i].sram.read_current_location(),
            Info.VEL:   lambda i: self.servo[i].sram.read_current_speed(),
            Info.LOAD:  lambda i: self.servo[i].sram.read_current_load(),
            Info.TEMP:  lambda i: self.servo[i].sram.read_current_temperature(),
            Info.VOLT:  lambda i: self.servo[i].sram.read_current_voltage(),
            Info.CURR:  lambda i: self.servo[i].sram.read_current_current(),
        }

        try:
            reader = readers[info_type]
        except KeyError:
            raise ValueError(f"Unknown info type: {info_type}")

        info_list = np.zeros(len(indices))
        for k, i in enumerate(indices):
            info_list[k] = reader(i)

        return info_list

    def set_motor_mode(self, mode, index):
        self.servo[index].eeprom.write_operating_mode(mode.value)

    def set_mode(self, modes=[MotorMode.POS], indices=None):
        indices = self.__no_indices(indices)
        if (len(modes) == 1):
            for index in indices:
                self.set_motor_mode(*modes, index)
        else:
            assert(len(indices) == len(modes) and "Length of the indices and modes must match")
            for k, (mode, index) in enumerate(modes, indices):
                self.set_motor_mode(mode, index)

    def set_motor_pos(self, pos, index):
        self.servo[index].sram.write_target_location(int(self.direction[index] * pos + self.pos_zero[index]))
        # print("Position command: ", int(self.direction[index] * pos + self.pos_zero[index]))


    def target_pos_step(self, poss, indices=None):
        indices = self.__no_indices(indices)
        assert(len(poss) == len(indices) and "Length of lists must match")
        for k, (pos, index) in enumerate(zip(poss, indices)):
            self.set_motor_pos(pos, index)

    def target_pos_deg(self, poss, indices=None):
        self.target_pos_step((np.array(poss)/self.pos_res).astype(int), indices)
    
    def target_pos_rad(self, poss, indices=None):
        self.target_pos_deg(np.rad2deg(poss), indices)
    
    def set_zero_pos(self, zero_pos):
        assert( (len(zero_pos) == self.numActuator))
        self.pos_zero = np.array(zero_pos)
    
    def set_direction(self, direction):
        assert( (len(direction) == self.numActuator))
        self.direction = direction
