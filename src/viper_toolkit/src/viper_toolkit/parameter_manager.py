#!/usr/bin/env python
# 
# VIPER: Vision Inference Processing (Edge) Endogenous Robot
# Andrew D'Amico
# MSDS 462 Computer Vision
# Northwestern University
# Copyright (c) 2021, Andrew D'Amico. All rights reserved.
# Licenced under BSD Licence.

import rospy
from viper_toolkit import NameManager, Logger



class Parameter(object):
    ''' 
    The parameter object gets, sets, and syncs a local and global param.
    
    Keyword arguments:
    name -- string indicating the name of the variable
    target -- string indicating the target address of the parameter, 
        generally in the form of '/node/name'
    default -- the default value, or starting value, for this parameter.
    dynamic -- bool indicating if the parameter should be updated automatically
    name_manager -- a NameManager object or pointer
    logger -- a Logger Object or pointer
    '''
    
    def __init__(self, 
                name: str = None, 
                target: str = None,
                dynamic: bool = False, 
                default: any = None,
                name_manager: NameManager = None,
                #parameters = None,
                logger: Logger = None,
                value: any = None):
                    self.setup_name_manager(var=name_manager)
                    self.setup_logger(var=logger)
                    self._name = name
                    self._target = target
                    self._default = default
                    self._dynamic = dynamic,
                    self.value = value

    def setup_name_manager(self, var=None):
        if var == None:
            self.name_manager = NameManager()
        else:
            self.name_manager = var
            
    def setup_logger(self, var=None):
        if var == None:
            self.logger = Logger(name_manager = self.name_manager)
        else:
            self.logger = var

    @property
    def name(self):
        return self._name
    @name.setter
    def name(self, var = None):
        if var == None:
            try:
                self._name = str(self.target.split('/')[-1])
                self.logger.i(f"Parameter name from target is {self._name}")
            except (NameError):
                self.logger.e("Target not defined.")
                self._name = self.name_manager.name
        else:
            self._name = str(var)
            self.logger.i(f'Passed Parameter name is {self._name}')
    
    @property
    def target(self):
        if self._target == None:
            self.target = None
        return self._target
    @target.setter
    def target(self, var=None):
        if var == None:
            try:
                my_location = self.name_manager.name
            except (NameError):
                self.logger.e("Name Manager not setup")
                try:
                    my_location = rospy.get_name()
                except (NameError):
                    self.logger.e("Target not defined.")
                    try:
                        my_location = rospy.search_param(self.name)[0]
                    except:
                        self.logger.e("Cannot deduce target")
                        return
            self._target = f'{my_location}/{self.name}'
            self.logger.i(f"Parameter target is {self._target}")
        else:
            self._target = var
            self.logger.i(f"Parameter target is {self._target}")
    @property
    def default(self):
        if self._default == None:
            self.default = None
        return self._default
    @default.setter
    def default (self, var=None):
        if var == None:
            if self.value != None:
                self._default = self.value
            else:
                try:
                    self._default = self.get_parameter(self.target)
                    self.logger.i("No default value given. Setting default to parameter.")
                except:
                    self.logger.e("No default value given, and no global parameter set.")
        else:
            #if self._value == None:
            #    self._value = var
            self._default = var
    @property
    def dynamic(self):
        if self._dynamic == None:
            self.dynamic = None
        return self._dynamic
    @dynamic.setter
    def dynamic(self, var: bool = False):
        self._dynamic = var
    
    def get_parameter(self, target):
        if rospy.has_param(target):
            self.logger.i(f'Parameter {self.name} found. Retreiving.')
            return rospy.get_param(target)
        else:
            pass

    def set_parameter(self, target, value):
        rospy.set_param(target, value)
        self.logger.i(f'Parameter {self.name} has been set to {self.value}.')
        
    def refresh_parameter(self):
        if self.dynamic:
            self.value = self.get_parameter(self.target)
        else:
            pass
            
    @property
    def value(self):
        return self._value
    @value.setter
    def value(self, var=None):
        if var == None:
            if self.dynamic:
                if rospy.has_param(self.target):
                    if self.get_parameter(self.target) != None:
                        self._value = self.get_parameter(self.target)
                        self.logger.i(f"Dynamic retrieval. Using {self.name} is {self._value}.")
                    else:
                        self._value = self.default
                else:
                    self._value = self.default
            else:
                self._value = self.default
                self.logger.i("Dynamic retrieval turned off. Using Default.")
        else:
            self._value = var


class ParameterManager(object):
    def __init__(self, 
                name_manager: NameManager = None, 
                logger: Logger = None):
                    self.setup_name_manager(var=name_manager)
                    self.setup_logger(var=logger)
                    self._parameters = {}

    def setup_name_manager(self, var = None):
        if var == None:
            try:
                self.name_manager = self.logger.name_manager
            except:
                self.name_manager = NameManager(name=rospy.get_name())
        else:
            self.name_manager = var
            
    def setup_logger(self, var = None):
        # If we are not given a logger, we will create one using our 
        # Name manager. Note that if the Name Manager was created
        # automatically it already has a '.' in its name, which 
        # indicates this is a parameter.
        if var == None:
            self.logger = Logger(name_manager=self.name_manager)
        else:
            self.logger = var

    def add(self, var):
        variable = var.name
        self.logger.i(f"Parameter name is {variable}")
        self._parameters[variable] = var
        setattr(self, variable, self._parameters[variable].value)
        self.logger.i(f"Parameter {variable} added.")

    def update(self):
        count = 0
        for parameter in self._parameters:
            if self._parameters[parameter].dynamic:
                old = self._parameters[parameter]
                self._parameters[parameter].refresh_parameter()
                new = self._parameters[parameter]
                if old != new:
                    self.logger().i(
                        f'Parameter {parameter} updated to new values.')
                    count = count + 1
        self.logger.i(f'Parameters updated. {count} updates found.')
            
if __name__ == "__main__":
    Parameter()
    Parameters()
