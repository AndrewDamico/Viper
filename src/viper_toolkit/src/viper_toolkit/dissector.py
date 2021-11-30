#!/usr/bin/env python
# 
# VIPER: Vision Inference Processing (Edge) Endogenous Robot
# Andrew D'Amico
# MSDS 462 Computer Vision
# Northwestern University
# Copyright (c) 2021, Andrew D'Amico. All rights reserved.
# Licenced under BSD Licence.
import inspect

def Dissect(method, class_):
    '''
    Dissect() takes a method and a class, and returns the contents of the function.
    
    Keyword Arguments:
    
    method -- the name of the method being invoked
    class_ -- the name of the class object dissected
    '''
    instance_ = class_
    i = f'instance_.{method}'
    print(f'class {class_.__name__}(object):')
    print("")
    e = eval(i)
    print(inspect.getsource(e))
    
if __name__ == "__main__":
    Dissect()
