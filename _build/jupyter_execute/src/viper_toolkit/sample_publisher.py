#!/usr/bin/env python
# coding: utf-8

# # Sample Publisher Nodes

# We can look at a two very simply sample nodes to understand how ROS works. 
# 
# In the Viper Toolkit package, under the scripts/ folder there are two python files:
#     * publisher.py
#     * subscriber.py
# 
# In ROS I find it easiest to set everything up as a class... In other notebooks I will try to dissect my classes, but for now lets take a look at these two nodes:

# In[1]:


import inspect
import rospy


# ## The Example Subscriber Node and Class Definition

# In[2]:


from script import publisher
publisher_source = inspect.getsource(publisher)
print (publisher_source)


# Lets take a look at the different parts of this node!

# ## Detailed Look at Class Attributes, Properties, and Methods

# In[3]:


from script.publisher import PublisherNode


# In[4]:


instance = PublisherNode


# In[5]:


def dissect(method, class_=SubscriberNode):
    '''
    Dissect() takes a method and a class, and returns the contents of the function.
    
    Keyword Arguments:
    
    method -- the name of the method being invoked
    class_ -- the name of the class object dissected
    '''
    instance_ = class_
    i = f'instance_.{method}'
    print(f'class {SubscriberNode.__name__}(object):')
    print("")
    e = eval(i)
    print(inspect.getsource(e))


# ### Initialize the Class Object
# The __init__ class function executes at the time that the class in instantiated. We are telling the interpreter to first run the setup_ros() function, then run the setup_publisher() function, and finally to execute the main_loop() function.

# In[6]:


dissect('__init__')


# ### 2. Setup ROS
# We need to tell ROS that we would like to bring this node online; we are registering this kernel as "sample_publisher". While multiple nodes of the same class can run at once, each node must have a unique name. If a node comes online which has the same name, it will replace the current node and this kernel will be shut down.

# In[7]:


dissect('setup_ros')


# ### 3. Setting up the Publisher
# Next we will setup the publisher API which allows a node to publish a "Topic", or a broadcasted message. This node will be publishing the topic "topic" a  message type of "String." It will only hold on to the last message (queue_size=1).

# In[8]:


dissect('setup_publisher')


# We can take a look at this message type. Messages of the same type need to be serialized so that a reciever is always receiving the same uniform format:

# In[9]:


from std_msgs.msg import String
message = String()
print (f"The class structure of String() is:")
print (message)
print ("")
print (f"The class type is: {type(message.data)}")


# This tells us that the message type String contains a single variable, 'data', which is of the class "str". ROS will attempt to convert classes between C++ and Python.

# ### 4. The Action Loop
# The Action Loop is where the processing of the node occures. In many of my nodes you will see multiple loops chained together, depending on the desired action to be performed. In this example our action loop is getting the time from the rospy module.

# In[10]:


dissect('action_loop')


# ### 5. Main Loop
# The main loop is the loop which will continually run while the node is online. First, we set the rospy.rate() which is the minimum hz we wish the node to run. In this example, we are asking the node to retrieve the result of the action loop (the time), converting our desired message into the 'String' message format, and then publishing our message. After printing a log entry indicating that it published a message it will then wait wait for the Hz timer to finish before it performs this loop again.

# In[11]:


dissect('main_loop')


# ## Final Output
# 
# Our resulting output looks like this.

# In[2]:


print ('''
if __name__ == '__main__':
    PublisherNode()

[INFO] [1638135063.888976]: I published: data: "The time is now 1638135063.8869355"
[INFO] [1638135063.988257]: I published: data: "The time is now 1638135063.9871924"
[INFO] [1638135064.088094]: I published: data: "The time is now 1638135064.0870602"
[INFO] [1638135064.188143]: I published: data: "The time is now 1638135064.1871343"
''')


# In[ ]:




