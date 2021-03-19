In case of error use services :

'rosservice call rosservice call /schunk_pg70/acknowledge_error'

And then the service to get new reference, be aware that after the call the robot will move to the "0" width, in order to avoid damage remove any possible obstacle between the fingers and eventually the fingers itself

'rosservice call /schunk_pg70/reference "{}" '

After that the gripper should be ready again to operate correclty.