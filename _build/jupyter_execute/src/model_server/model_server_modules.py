#!/usr/bin/env python
# coding: utf-8

# In[1]:


import inspect


# In[2]:


from model_server.msg import InferenceResults
lines = inspect.getsource(InferenceResults)
print(lines)


# In[10]:


from model_server.srv import ImageRequest, ImageRequestResponse
lines = inspect.getsource(ImageRequest)
print(lines)


# In[11]:


lines = inspect.getsource(ImageRequestResponse)
print(lines)


# In[12]:


from model_server.srv import ModelRequest, ModelRequestResponse
lines = inspect.getsource(ModelRequest)
print(lines)


# In[13]:


lines = inspect.getsource(ModelRequestResponse)
print(lines)


# In[ ]:





# In[5]:


from model_server import ViperModel
lines = inspect.getsource(ViperModel)
print(lines)


# In[3]:


from model_server import NeuralNetworkLoader
lines = inspect.getsource(NeuralNetworkLoader)
print(lines)


# In[ ]:




