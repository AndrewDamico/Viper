#!/usr/bin/env python
# 
# VIPER: Vision Inference Processing (Edge) Endogenous Robot
# Andrew D'Amico
# MSDS 462 Computer Vision
# Northwestern University
# Copyright (c) 2021, Andrew D'Amico. All rights reserved.
# Licenced under BSD Licence.


import time
import rospy
from viper_toolkit import Logger, NameManager


class ProcessTimer(object):
    def __init__(self, abv = None, name = None, logger = None):
        self.record = {}
        self._abv = abv
        self._log(tag = "total_runtime")
        self._logger = logger
        self._name = name
    @property
    def logger(self):
        return self._logger
    @logger.setter
    def logger(self, value):
        self._logger = value
    @property
    def abv(self):
        return self._abv
    @abv.setter
    def abv(self, value = None):
        if value == None:
            if self._logger != None:
                if type(self._logger) == Logger:
                    self._abv = self._logger.tag
            elif self._name != None:
                    if type(self._name) == NameManager:
                        self._abv = self._name.abv
        else:
            self._abv = value
    def _log(self, tag: str):
        self.record[tag] = {"start": time.time(),
                            "end" : 0,
                            "total": 0}
    def lap(self, tag:str):
        self._log(tag)
    def time(self, tag:str, name:str = None):
        end = time.time()
        try:
            start = self.record[tag]['start']
            self.record[tag]['end'] = end
            total_time = self.ms(start, end)
            self.record[tag]['total'] = total_time
            self.log_info(f"{tag} time: {total_time}ms", name)
        except:
            self.log_error(f'[name] {tag} not found.', name)
            return
    def log_info(self, message, name):
        if type(self._logger) == Logger:
            self.logger.i(message, name)
        else:
            if name == None:
                rospy.loginfo(f'[{self._abv}] {message}')
            else:
                rospy.loginfo(f'[{name}] {message}')
    def log_error(self, message, name):
        if type(self._logger) == Logger:
            self.logger.e(message, name)
        elif name != None:
            rospy.logerr(f'[{name}] {message}')
        else:
            rospy.logerr(f'[{self._abv}] {message}')

    def ms(self, start, end):
        return int(1000*(end - start))
        

if __name__ == "__main__":
    ProcessTimer()
