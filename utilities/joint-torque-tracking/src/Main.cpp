/**
 * @file Main.cpp
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

// YARP
#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>

#include <BipedalLocomotion/JointTorqueTracking/Module.h>

int main(int argc, char* argv[])
{
    // initialize yarp network
    yarp::os::Network yarp;
    if (!yarp.checkNetwork())
    {
        yError() << "[main] Unable to find YARP network";
        return EXIT_FAILURE;
    }

    // prepare and configure the resource finder
    yarp::os::ResourceFinder& rf = yarp::os::ResourceFinder::getResourceFinderSingleton();

    rf.setDefaultConfigFile("jointTorqueTrackingOptions.ini");

    rf.configure(argc, argv);

    // create the module
    BipedalLocomotion::JointTorqueTracking::Module module;

    return module.runModule(rf);
}
