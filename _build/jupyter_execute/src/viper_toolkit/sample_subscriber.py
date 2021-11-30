#!/usr/bin/env python
# coding: utf-8

# # Sample Subscriber Nodes

# The subscriber node is very similar to the Publisher node, and many of my nodes are both publishers and subscribers. 
# 
# The key difference is that the publisher and subscriber functions have a one to many relationship: there can be multiple subscribers subscribed to a topic, but there can only be one publisher.
# 
# A subscriber node receives the messages published by the subscriber nodes, up to the number of messages allowed in its queue.

# In[1]:


import inspect
import rospy
from std_msgs.msg import String


# ## The Example Subscriber Node and Class Definition
# In each package, located in the "viper/src" director of my git repository, there is a 'script' folder. Within this folder are the various nodes for that package (which usually are also a class definition of that node). We can take a look at the sample subscriber node I built within the "viper_toolkit" package.

# In[2]:


from script import subscriber
subscriber_source = inspect.getsource(subscriber)
print (subscriber_source)


# Each of my class definitions will look similar, although will greatly vary in complexity. 
# 
# Lets take a look at the different parts of this node.

# ## Detailed Look at Class Attributes, Properties, and Methods

# In[3]:


from script.subscriber import SubscriberNode


# In[4]:


instance = SubscriberNode
instance.message = String("The time is now 1638136508.7424648")


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


# In[6]:


dissect('__init__')


# ### Initialize the Class Object
# The __init__ class function executes at the time that the class in instantiated. We are telling the interpreter to first run the setup_ros() function, then run the setup_publisher() function, and finally to execute the main_loop() function.

# In[7]:


dissect('__init__')


# ### Setup ROS
# We need to tell ROS that we would like to bring this node online; we are registering this kernel as "sample_subscriber". 
# 
# During this step we will also set the class "message" attribute using the class String.This allows us to retrieve the previous message upon failure so that we do not disrupt the system loop in the event of a dropped message. Note that the message types between the subscribers and the publishers must be the same.

# In[8]:


dissect('setup_ros')


# ### Setting up the Publisher
# Next we will setup the subscriber API which allows a node to subscribe to a "Topic." This node will be receive all messages of the topic "topic", but will only hold on to the last message (queue_size=1).
# 
# When it receives a new message it will execute its class callback() function

# In[9]:


dissect('setup_subscriber')


# ### The Callback Function
# The callback() function tells the node what to do once it receives a message. In this example we are asking the node to save the incoming message to the container we set up earlier. Rather than perform an action in the main loop, which is activated cyclically, we will have our action originate from this callback.
# 
# Thoughout my modules I use a variety of types of messages and callbacks to not only pass on data such as images and inference results, but to also publish "flags" or indicators that a node wants something.

# In[10]:


dissect('callback')


# We can take a look at the message received:

# In[11]:


print ("Sample Message Received")
print (instance.message)


# ### The Action Loop
# In our example action loop we will take the most recent message and transform its case into capital letters. We access the data in the message through it message.data class property. We then output the results to the terminal, as well as return the results when this function is called.

# In[12]:


dissect('action_loop')


# ### Main Loop
# Like the publisher node, the main loop is the loop which will continually run while the node is online. Since all of our actions are occuring within the callback loop we do not have much in our main loop, other than the rospy.spin() function which keeps the kernal alive in the case that it is not actively processing data (but still waiting for messages).

# In[13]:


dissect('main_loop')


# ## Final Output
# 
# Our resulting output looks like this.

# In[14]:


print ('''
if __name__ == '__main__':
    SubscriberNode()

[INFO] [1638136507.446916]: I heard The time is now 1638136507.4424667
[INFO] [1638136507.549606]: I heard The time is now 1638136507.542628
[INFO] [1638136507.645125]: I heard The time is now 1638136507.6427877
[INFO] [1638136507.744053]: I heard The time is now 1638136507.7424889
''')


# In[ ]:




