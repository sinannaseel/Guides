Obtain the the dh parameters online from the websites

eg: UR3e has the model dh parameters

Things to check : 

1. Reference frame (or base frame) set is what you think you want your datum as, make sure it is similar to the model dh parameters. 
2. Create spheres(and frames) or object to simulate the forward kinematics onto the simulation 
3. The fkm showing the spheres and the actual robot willl be completely different
4. Usually theta isnt given because you will have to set it up yourself, pay heed to the dimensions of length(d and a) and not angle(alpha and theta). check if everyjoint is matching, you can change the value of theta one joint at a time, starting from the first joint.
5. To calibrate it to precision you need o use least squares method for minute errors (will come in handy during grasping).

Always remember (it is quite easy there it might feel sometimes), there is brunos book Robotics modelling, planning and control. if you want to check the values.
