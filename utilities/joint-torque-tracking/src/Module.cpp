/**
 * @file Module.cpp
 * @copyright 2024 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */


#include <chrono>
#include <fstream>
#include <iomanip>

#include <BipedalLocomotion/ParametersHandler/YarpImplementation.h>
#include <BipedalLocomotion/RobotInterface/YarpHelper.h>
#include <BipedalLocomotion/RobotInterface/YarpRobotControl.h>
#include <BipedalLocomotion/RobotInterface/YarpSensorBridge.h>

#include <Eigen/Dense>

#include <BipedalLocomotion/JointTorqueTracking/Module.h>

#include <yarp/dev/IEncoders.h>


using Vector1d = Eigen::Matrix<double, 1, 1>;

using namespace BipedalLocomotion;
using namespace BipedalLocomotion::JointTorqueTracking;

double Module::getPeriod()
{
    return std::chrono::duration<double>(m_dT).count();
}

bool Module::createPolydriver(std::shared_ptr<ParametersHandler::IParametersHandler> handler)
{
    constexpr auto logPrefix = "[Module::createPolydriver]";
    auto ptr = handler->getGroup("ROBOT_INTERFACE").lock();
    if (ptr == nullptr)
    {
        log()->error("{} Unable to find the group ROBOT_INTERFACE.", logPrefix);
        return false;
    }
    ptr->setParameter("local_prefix", this->getName());
    m_controlBoard = RobotInterface::constructRemoteControlBoardRemapper(ptr);
    if (!m_controlBoard.isValid())
    {
        log()->error("{} Unable to create the polydriver.", logPrefix);
        return false;
    }

    // check the number of controlled joints
    int controlBoardDOFs = 0;
    yarp::dev::IEncoders* axis;
    m_controlBoard.poly->view(axis);
    if (axis != nullptr)
    {
        axis->getAxes(&controlBoardDOFs);
    }

    if (controlBoardDOFs != 1)
    {
        log()->error("{} The number of controlled joints is not equal to 1.", logPrefix);
        return false;
    }

    return true;
}

bool Module::initializeRobotControl(std::shared_ptr<ParametersHandler::IParametersHandler> handler)
{
    if (!m_robotControl.initialize(handler->getGroup("ROBOT_CONTROL")))
    {
        log()->error("[Module::initializeRobotControl] Unable to initialize the control board");
        return false;
    }
    if (!m_robotControl.setDriver(m_controlBoard.poly))
    {
        log()->error("[Module::initializeRobotControl] Unable to set the driver");
        return false;
    }

    return true;
}

bool Module::instantiateSensorBridge(std::shared_ptr<ParametersHandler::IParametersHandler> handler)
{
    if (!m_sensorBridge.initialize(handler->getGroup("SENSOR_BRIDGE")))
    {
        log()->error("[Module::initializeSensorBridge] Unable to initialize the sensor bridge");
        return false;
    }

    yarp::dev::PolyDriverList list;
    list.push(m_controlBoard.poly.get(), m_controlBoard.key.c_str());
    if (!m_sensorBridge.setDriversList(list))
    {
        log()->error("[Module::initializeSensorBridge] Unable to set the driver list");
        return false;
    }

    return true;
}



bool Module::configure(yarp::os::ResourceFinder& rf)
{
    m_currentJointPos.resize(1);

    auto parametersHandler = std::make_shared<ParametersHandler::YarpImplementation>(rf);

    std::string name;
    if (!parametersHandler->getParameter("name", name))
    {
        log()->error("[Module::configure] Unable to find the parameter 'name'.");
        return false;
    }

    if (!parametersHandler->getParameter("sampling_time", m_dT))
    {
        log()->error("[Module::configure] Unable to find the parameter 'sampling_time'.");
        return false;
    }

    double maxValue = 0;
    if (!parametersHandler->getParameter("max_angle_deg", maxValue))
    {
        log()->error("[Module::configure] Unable to find the parameter 'max_angle_deg'.");
        return false;
    }

    maxValue *= M_PI / 180;

    double minValue = 0;
    if (!parametersHandler->getParameter("min_angle_deg", minValue))
    {
        log()->error("[Module::configure] Unable to find the parameter 'min_angle_deg'.");
        return false;
    }

    minValue *= M_PI / 180;

    double trajectoryDuration = 5;
    if (!parametersHandler->getParameter("trajectory_duration", trajectoryDuration))
    {
        log()->error("[Module::configure] Unable to find the parameter 'trajectory_duration'.");
        return false;
    }

    if (!parametersHandler->getParameter("refernce_torque_kp", m_kp))
    {
        log()->error("[Module::configure] Unable to find the parameter 'kp'.");
        return false;
    }

    if (!parametersHandler->getParameter("refernce_torque_kd", m_kd))
    {
        log()->error("[Module::configure] Unable to find the parameter 'kd'.");
        return false;
    }

    if (!parametersHandler->getParameter("refernce_torque_ki", m_ki))
    {
        log()->error("[Module::configure] Unable to find the parameter 'ki'.");
        return false;
    }

    if (!parametersHandler->getParameter("torque_constant", m_torqueConstant))
    {
        log()->error("[Module::configure] Unable to find the parameter 'torque_constant'.");
        return false;
    }


    this->createPolydriver(parametersHandler);
    this->initializeRobotControl(parametersHandler);
    this->instantiateSensorBridge(parametersHandler);

    m_setPoints.push_back((maxValue + minValue) / 2);
    m_setPoints.push_back(maxValue);
    m_setPoints.push_back(minValue);
    m_setPoints.push_back((maxValue + minValue) / 2);

    m_spline.setAdvanceTimeStep(m_dT);
    m_spline.setInitialConditions(Eigen::VectorXd::Zero(1), Eigen::VectorXd::Zero(1));
    m_spline.setFinalConditions(Eigen::VectorXd::Zero(1), Eigen::VectorXd::Zero(1));

    m_timeKnots.clear();
    m_timeKnots.push_back(std::chrono::nanoseconds::zero());
    m_timeKnots.push_back(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(trajectoryDuration)));

    if (!m_sensorBridge.advance())
    {
        log()->error("[Module::configure] Unable to read the sensor.");
        return false;
    }
    m_sensorBridge.getJointPositions(m_currentJointPos);


    m_setPoints.push_back(m_currentJointPos[0]);
    m_currentSetPoint = m_setPoints.begin();

    m_trajectoryKnots.push_back(m_currentJointPos);
    m_trajectoryKnots.push_back(Vector1d::Constant(*m_currentSetPoint));

    m_spline.setKnots(m_trajectoryKnots, m_timeKnots);

    m_initTrajectoryTime = yarp::os::Time::now();

    // switch the torque control mode
    if (!m_robotControl.setControlMode(RobotInterface::ControlMode::Torque))
    {
        log()->error("[Module::generateNewTrajectory] Unable to switch to torque control mode.");
        return false;
    }


    return true;
}

bool Module::generateNewTrajectory()
{
    double initTrajectory = *m_currentSetPoint;

    // the trajectory is ended
    if (std::next(m_currentSetPoint, 1) == m_setPoints.end())
    {
        return false;
    }

    std::advance(m_currentSetPoint, 1);
    double endTrajectory = *m_currentSetPoint;

    m_trajectoryKnots[0] = Vector1d::Constant(initTrajectory);
    m_trajectoryKnots[1] = Vector1d::Constant(endTrajectory);

    m_spline.setKnots(m_trajectoryKnots, m_timeKnots);

    m_initTrajectoryTime = yarp::os::Time::now();

    return true;
}

bool Module::updateModule()
{
    if (!m_sensorBridge.advance())
    {
        log()->error("[Module::updateModule] Unable to read the sensor.");
        return false;
    }

    m_sensorBridge.getJointPositions(m_currentJointPos);

    if (yarp::os::Time::now() - m_initTrajectoryTime > m_spline.getFinalTime())
    {
        if (!generateNewTrajectory())
        {
            return false;
        }
    }

    double positionError = m_spline.getOutput().position[0] - m_currentJointPos[0];


    // PID controller for position to generate desired torque

    m_integral += positionError * m_dT;
    double derivative = (positionError - m_previousError) / m_dT;
    double desiredTorque = m_kp * positionError + m_ki * m_integral + m_kd * derivative;
    m_previousError = positionError;

    // set the reference
    if (!m_robotControl.setReferences(Eigen::VectorXd::Constant(1, desiredTorque),
                                      RobotInterface::ControlMode::Torque,
                                      m_currentJointPos))
    {
        log()->error("[Module::updateModule] Unable to set the reference.");
        return false;
    }

    m_logDesiredJointTorque.push_back(desiredTorque);

    m_previousError = positionError;

    m_logJointPos.push_back(m_currentJointPos[0]);
    m_logDesiredJointPos.push_back(m_spline.getOutput().position[0]);

    // Advance the spline
    if (!m_spline.advance())
    {
        log()->error("[Module::updateModule] Unable to advance the spline.");
        return false;
    }

    const double now = yarp::os::Time::now();
    if (now - m_initTrajectoryTime > std::chrono::duration<double>(m_timeKnots.back()).count() + 2)
    {
        std::cout << "[Module::updateModule] Generate a new trajectory." << std::endl;

        if (!generateNewTrajectory())
        {
            std::cerr << "[Module::updateModule] Experiment finished." << std::endl;
            return false;
        }
    }

    return true;
}


bool Module::close()
{
    std::ofstream file("log.txt");
    if (!file.is_open())
    {
        log()->error("[Module::close] Unable to open the file.");
        return false;
    }

    file << std::fixed << std::setprecision(5);
    for (size_t i = 0; i < m_logJointPos.size(); i++)
    {
        file << m_logJointPos[i] << " " << m_logDesiredJointPos[i] << " " << m_logDesiredJointTorque[i]
             << std::endl;
    }

    file.close();

    return true;
}
