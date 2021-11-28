#!/usr/bin/env python
# 
# VIPER: Vision Inference Processing (Edge) Endogenous Robot
# Andrew D'Amico
# MSDS 462 Computer Vision
# Northwestern University
# Copyright (c) 2021, Andrew D'Amico. All rights reserved.
# Licenced under BSD Licence.

import os
import rospy

class NameManager(object):
    """Creates a NameManager class object which is used to standardize
    the names for a node or a process, and is linked to other 
    Viper toolkit modules, such as the logger. It includes rules for 
    automatically setting the name if a name isn't provided in order
    to standardize log messages, parameters, etc.
    
    Keyword arguments:
    abv -- the abreviated name for the object
    name -- the full name for the object
    """
    def __init__(self, name: str = None, abv: str = None):
        # Sets the name of this module for the sake of logging.
        self._code = "NMG"
        # Retrieves the name of the module and sets to self._pkg_name
        self._name = name
        self._abv = abv
    @property
    def name(self):
        return self._name
    @name.setter
    def name(self, name: str = None):
        if name == None:
            try:
                self._name = f'{rospy.get_name()}'[1:]
            except:
                self._name = "ERROR"
        else:
            self._name = name
        rospy.logdebug(f'[{self._code}] Name set to %s', self.name)
        #setattr(self, name, self._name)
    @property
    def abv(self):
        return self._abv
    @abv.setter
    def abv(self, code = None):
        if code == None:
            self._abv = self._autoabv()
        else:
            self._abv = self.codeformatter(code)
        rospy.set_param(f'/{self.name}/abv', self.abv)
        rospy.logdebug(f'[{self._code}] ABV set')
    @classmethod
    def codeformatter(self, code: str):
        abv = code[:3].upper()
        abv = abv.ljust(3)
        return abv
    @classmethod
    def _autoabv(self):
        if rospy.has_param(f'{self.name}/abv'):
            abv = rospy.get_param(f'/{self.name}/abv')
            return self.codeformatter(abv)
        else:
            return self.codeformatter(f'{self.name}')


if __name__ == "__main__":
    NameManager()
