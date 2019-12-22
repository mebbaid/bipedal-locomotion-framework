/**
 * @file OptimizationProblemTest.cpp
 * @authors Giulio Romualdi
 * @copyright 2019 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

// Catch2
#include <catch2/catch.hpp>

#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/Model/ModelTestUtils.h>

#include <BipedalLocomotionControllers/OptimalControlUtilities/CartesianElements.h>
#include <BipedalLocomotionControllers/OptimalControlUtilities/OptimizationProblemElements.h>
#include <BipedalLocomotionControllers/OptimalControlUtilities/RegularizationElements.h>
#include <BipedalLocomotionControllers/OptimalControlUtilities/VariableHandler.h>


using namespace BipedalLocomotionControllers::OptimalControlUtilities;

TEST_CASE("Optimization problem elements")
{
    unsigned int numberOfJoints = 20;

    iDynTree::Model model = iDynTree::getRandomModel(numberOfJoints, 10);
    unsigned int numberDoFs = model.getNrOfDOFs();

    std::shared_ptr<iDynTree::KinDynComputations> kinDyn;
    kinDyn = std::make_shared<iDynTree::KinDynComputations>();

    kinDyn->loadRobotModel(model);
    kinDyn->setFrameVelocityRepresentation(iDynTree::MIXED_REPRESENTATION);
    REQUIRE(kinDyn->setFloatingBase("baseLink"));

    iDynTree::Vector3 gravity;
    gravity.zero();
    gravity(2) = -9.81;
    iDynTree::VectorDynSize s(numberDoFs);
    iDynTree::toEigen(s).setRandom();

    iDynTree::VectorDynSize ds(numberDoFs);
    iDynTree::toEigen(ds).setRandom();

    REQUIRE(kinDyn->setRobotState(iDynTree::Transform::Identity(),
                                  s,
                                  iDynTree::Twist::Zero(),
                                  ds,
                                  gravity));

    std::string linkInContact1 = "link5";
    std::string linkInContact2 = "link9";

    VariableHandler handler;

    handler.addVariable("base_acceleration", 6);
    handler.addVariable("joint_accelerations", numberDoFs);
    handler.addVariable("joint_torques", numberDoFs);
    handler.addVariable("link_in_contact_1", 6);
    handler.addVariable("link_in_contact_2", 6);

    Constraints constraints(handler);

    auto comElement = std::make_unique<CartesianElement>(kinDyn,
                                                         handler,
                                                         CartesianElement::Type::POSITION,
                                                         "CoM");
    auto leftFootElement = std::make_unique<CartesianElement>(kinDyn,
                                                              handler,
                                                              CartesianElement::Type::POSE,
                                                              linkInContact1);
    auto rightFootElement = std::make_unique<CartesianElement>(kinDyn,
                                                               handler,
                                                               CartesianElement::Type::POSE,
                                                               linkInContact2);
    constraints.addConstraint(comElement.get());
    constraints.addConstraint(leftFootElement.get());
    constraints.addConstraint(rightFootElement.get());
    REQUIRE(comElement);

    iDynTree::Vector3 gains;
    for (unsigned int i = 0; i < 3; i++)
        gains(i) = 1;

    comElement->setLinearPDGains(gains, gains);

    iDynTree::Vector3 dummy;
    dummy.zero();

    comElement->setDesiredTrajectory(dummy, dummy, dummy);

    Constraints::Bounds bounds = constraints.getBounds();
    std::cerr << "Lower bounds " << bounds.lowerBound().toString() << std::endl;
    std::cerr << "Upper bounds " << bounds.upperBound().toString() << std::endl;

    std::cerr << "Constraint matrix " << constraints.getConstraintMatrix().toString() << std::endl;

    auto jointTorquesRegularization
        = std::make_unique<RegularizationElement>(kinDyn, handler, "joint_torques");

    CostFunction costFunction(handler);
    iDynTree::VectorDynSize weight(numberDoFs);

    for (int i = 0; i < weight.size(); i++)
        weight(i) = 1;

    costFunction.addCostFunction(jointTorquesRegularization.get(),
                                 weight,
                                 1,
                                 0,
                                 "joint_regularization");

    CostFunction::Elements elements = costFunction.getElements();

    std::cerr << "Hessian " << elements.hessian().toString() << std::endl;
    std::cerr << "Gradient " << elements.gradient().toString() << std::endl;
}
