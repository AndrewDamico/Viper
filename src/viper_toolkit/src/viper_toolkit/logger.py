#!/usr/bin/env python
# 
# VIPER: Vision Inference Processing (Edge) Endogenous Robot
# Andrew D'Amico
# MSDS 462 Computer Vision
# Northwestern University
# Copyright (c) 2021, Andrew D'Amico. All rights reserved.
# Licenced under BSD Licence.

import rospy
from . import NameManager

class Logger(object):
    """ The Logger object managers the format for the logging output
    of various viper nodes and modules. It creates a standardized format
    used between the Logger, the Parameter manager, the name manager, 
    etc.
    
    Keyword Arguments:
    name -- the name or name manager to be used by the logger
    tag -- the default [tag] to be used by the logger
    """
    def __init__(self, 
                name_manager: NameManager = None, 
                name: str = None, 
                tag: str = None):
                    self._name = name
                    self._tag = tag
                    self.setup_name_manager(name_manager)
                    
    def setup_name_manager(self, var=None):
        if var == None:
            if self.name == None:
                if self.tag == None:
                    self.name_manager = NameManager()
                    self.tag = self.name_manager.abv
                else:
                    self.name_manager = NameManager(name=self.tag, abv=self.tag)
                self.name = self.name_manaager.name
            else:
                self.name_manager = NameManager(name=self.name)
        else:
            self.name_manager = var
            self.name = self.name_manager.name
   
    def set_name(self):
        self.name_manager.name
    @property
    def name(self):
        return self._name
    @name.setter
    def name(self, var=None):
        if var == None:
                self._name = self.name_manager.name
        else:
            self._name = var
    @property
    def tag(self):
        if self._tag == None:
            self.tag = None
        return self._tag
    @tag.setter
    def tag(self, var=None):
        if var == None:
            self._tag = self.name_manager.abv
        else:
            self._tag = var
    @classmethod
    def i(self, message: str , itag: str = None):
        ''' Rospy Information Log'''
        if itag == None:
            itag = self.tag
        m = self.message_formatter(message=message, itag=itag)
        return rospy.loginfo(m)
    @classmethod
    def e(self, message: str, itag: str = None):
        ''' Rospy Error Log'''
        if itag == None:
            itag = self.tag
        m = self.message_formatter(message=message, itag=itag)
        return rospy.logerr(m)
    @classmethod
    def d(self, message: str, itag: str = None):
        ''' Rospy Debug Log'''
        if itag == None:
            itag = self.tag
        m = self.message_formatter(message=message, itag=itag)
        return rospy.logdebug(m)
    @classmethod
    def message_formatter(self, message, itag = None):
        if itag == None:
            itag_o = self.tag
        else:
            itag_o = NameManager.codeformatter(f'{itag}')
        m = f"[{itag_o}] {message}"
        return m

if __name__ == "__main__":
    Logger()
