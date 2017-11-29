### ITIA_RUTILS DOCUMENTATION

utilities for ROS messages

author: manuel.beschi@itia.cnr.it
maintainer: enrico.villagrossi@itia.cnr.it

# USAGE

Here is an example of usage:

``` cpp
geometry_msgs::PoseStamped msg;
itia_rutils::MsgReceiver<geometry_msgs::PoseStamped> msg_receiver("PoseMsgReceived");
ros::Subscriber   sub=nh.subscribe("pose_state",
                                    1,&itia_rutils::MsgReceiver<geometry_msgs::PoseStamped>::callback,
                                    &msg_receiver);
if (!msg_receiver.waitFirstEntry(10))
{
  std::cout <<"error!"<<std::endl;
  return -1;
}
msg=msg_receiver.getData();

ros::Rate lp(10);
while (ros::ok())
{
  if (!msg_receiver.getNewData())
    std::cout << "no new data received!" << std::cout;
  else
  {
    msg=msg_receiver.getData();
  }
  lp.sleep();
  ros::spin();
}
``` 