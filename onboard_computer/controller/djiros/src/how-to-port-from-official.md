## How to Port Code from Official Onboard-SDK-ROS into Djiros ##
2017-06-16 v3.3

1. Copy **msg/**, **srv/**, **include/dji_sdk/**, **src/** from official into djiros

2. Edit **include/dji_sdk/dji_sdk_node.h**:
    * Modify all ```#include <dji_sdk/...>``` to ```#include <djiros/...>``` to make the messages and services compatible
    * Add ```namespace dji_sdk = djiros;``` to make the namespace of ros generated messages and services compatible
    * Add an empty constructor ```DJISDKNode(){};``` in ```class DJISDKNode``` for derived class to access all members
    * Modify all ```private:``` to ```protected:``` for derived class to access all members
    * Comment out **src/modules/dji_sdk_node.cpp**:L77-L89 
