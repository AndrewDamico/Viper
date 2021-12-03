#!/usr/bin/env python
# 
# VIPER: Vision Inference Processing (Edge) Endogenous Robot
# Andrew D'Amico
# MSDS 462 Computer Vision
# Northwestern University
# Copyright (c) 2021, Andrew D'Amico. All rights reserved.
# Licenced under BSD Licence.
import inspect

def Dissect(method=None, class_=None, package=None, ros=True, directory='./scripts'):
    '''
    Dissect() takes a method and a class, and returns the contents of the function.
    
    Keyword Arguments:
    
    method -- the name of the method being invoked
    class_ -- the name of the class object dissected
    package -- the name of the package we are getting the script from
    ros -- indicates if this is a ROS node package (without the .py extension)
    directory -- sets the package directory, if different from './scripts'
    '''
    
    instance_ = class_
    i = f'instance_.{method}'
    print(f'class {class_.__name__}(object):')
    print("")
    e = eval(i)
    print(inspect.getsource(e))
    
if __name__ == "__main__":
    Dissect()

