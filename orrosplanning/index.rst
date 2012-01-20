orrosplanning ROS Launch Files
==============================

**Description:** OpenRAVE Plugin for ROS Planning

  
      Contains robot ik solvers, planners, and commonly used functions that integrate with the ROS framework.
    

**License:** BSD

collada_rviz_display.launch
---------------------------

.. code-block:: bash

  roslaunch orrosplanning collada_rviz_display.launch


Loads a collada model onto the parameter server and publishes joint state information. The collada filename is passed as an argument to the launch script.

For example::

.. code-block:: bash

  roslaunch orrosplanning collada_rviz_display.launch model:=`rospack find collada_robots`/data/robots/kawada-hironx.dae

.. image:: collada_rviz_hiro.png
  :width: 400

  

Contents
########

.. code-block:: xml

  <launch>
    <arg name="model" />
    <param name="robot_description" textfile="$(arg model)" />
    <node name="collada_joint_publisher" pkg="orrosplanning" type="collada_joint_publisher.py" />
    <node args="-d $(find orrosplanning)/urdf.vcg" name="rviz" output="screen" pkg="rviz" type="rviz" />
    </launch>

testarmik5d.launch
------------------

.. code-block:: bash

  roslaunch orrosplanning testarmik5d.launch

