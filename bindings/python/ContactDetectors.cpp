/**
 * @file ContactDetectors.cpp
 * @authors Prashanth Ramadoss
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "BipedalLocomotion/ContactDetectors/ContactDetector.h"
#include "BipedalLocomotion/ContactDetectors/SchmittTriggerDetector.h"
#include "BipedalLocomotion/ContactDetectors/ContactBayesFilter.h"
#include "bipedal_locomotion_framework.h"


void BipedalLocomotion::bindings::CreateContactDetector(pybind11::module& module)
{
    namespace py = ::pybind11;
    using namespace BipedalLocomotion::Contacts;
    using namespace BipedalLocomotion::System;
    using namespace BipedalLocomotion::ParametersHandler;

    py::class_<Advanceable<EstimatedContactList>>(module, "EstimatedContactListAdvanceable");
    py::class_<ContactDetector, Advanceable<EstimatedContactList>>(module, "ContactDetector");
}

void BipedalLocomotion::bindings::CreateSchmittTriggerUnit(pybind11::module& module)
{
    namespace py = ::pybind11;
    using namespace BipedalLocomotion::Contacts;

    py::class_<SchmittTriggerInput>(module, "SchmittTriggerInput")
        .def(py::init())
        .def_readwrite("time", &SchmittTriggerInput::time)
        .def_readwrite("value", &SchmittTriggerInput::value);

    py::class_<SchmittTriggerParams>(module, "SchmittTriggerParams")
        .def(py::init())
        .def_readwrite("on_threshold", &SchmittTriggerParams::onThreshold)
        .def_readwrite("off_threshold", &SchmittTriggerParams::offThreshold)
        .def_readwrite("switch_on_after", &SchmittTriggerParams::switchOnAfter)
        .def_readwrite("switch_off_after", &SchmittTriggerParams::switchOffAfter);

    py::class_<SchmittTriggerUnit>(module, "SchmittTriggerUnit")
        .def(py::init())
        .def("set_state", py::overload_cast<const bool& >(&SchmittTriggerUnit::setState), py::arg("state"))
        .def("set_state", py::overload_cast<const bool&, const double& >(&SchmittTriggerUnit::setState), py::arg("state"), py::arg("time_now"))
        .def("set_params", &SchmittTriggerUnit::setParams, py::arg("params"))
        .def("update", &SchmittTriggerUnit::update, py::arg("current_time"), py::arg("raw_value"))
        .def("reset", &SchmittTriggerUnit::reset)
        .def("get_state", py::overload_cast<>(&SchmittTriggerUnit::getState))
        .def("get_state_and_switch_time",
             [](SchmittTriggerUnit& impl) {
                 double switchTime;
                 bool ok = impl.getState(switchTime);
                 return std::make_tuple(ok, switchTime);
             })
        .def("get_params", &SchmittTriggerUnit::getParams);
}

void BipedalLocomotion::bindings::CreateSchmittTriggerDetector(pybind11::module& module)
{
    namespace py = ::pybind11;
    using namespace BipedalLocomotion::Contacts;
    using namespace BipedalLocomotion::System;
    using namespace BipedalLocomotion::ParametersHandler;

    py::class_<SchmittTriggerDetector, ContactDetector>(module, "SchmittTriggerDetector")
        .def(py::init())
        .def(
            "initialize",
            [](SchmittTriggerDetector& impl, std::shared_ptr<IParametersHandler> handler) -> bool {
                return impl.initialize(handler);
            },
            py::arg("handler"))
        .def("advance", &SchmittTriggerDetector::advance)
        .def("reset_contacts", &SchmittTriggerDetector::resetContacts)
        .def("get", py::overload_cast<>(&SchmittTriggerDetector::get, py::const_))
        .def("get", py::overload_cast<const std::string&>(&SchmittTriggerDetector::get, py::const_),
             py::arg("contact_name"))
        .def("is_valid", &SchmittTriggerDetector::isValid)
        .def("set_timed_trigger_input", &SchmittTriggerDetector::setTimedTriggerInput,
             py::arg("contact_name"), py::arg("time"), py::arg("trigger_input"))
        .def("set_timed_trigger_inputs", &SchmittTriggerDetector::setTimedTriggerInputs,
             py::arg("timed_inputs"))
        .def("add_contact", py::overload_cast<const std::string&, const bool&, const SchmittTriggerParams&>(&SchmittTriggerDetector::addContact),
             py::arg("contact_name"), py::arg("initial_state"), py::arg("params"))
        .def("add_contact", py::overload_cast<const std::string&, const bool&, const SchmittTriggerParams&, const double&>(&SchmittTriggerDetector::addContact),
             py::arg("contact_name"), py::arg("initial_state"), py::arg("params"), py::arg("time_now"))
        .def("reset_state", &SchmittTriggerDetector::resetState,
            py::arg("contact_name"), py::arg("state"))
        .def("reset_contact", &SchmittTriggerDetector::resetContact,
            py::arg("contact_name"), py::arg("state"), py::arg("params"))
        .def("remove_contact", &SchmittTriggerDetector::removeContact,
            py::arg("contact_name"));
}

void BipedalLocomotion::bindings::CreateContactBayesFilter(pybind11::module& module)
{
    namespace py = ::pybind11;
    using namespace BipedalLocomotion::Contacts;
    using namespace BipedalLocomotion::System;
    using namespace BipedalLocomotion::ParametersHandler;

    py::class_<ContactBayesCollision>(module, "ContactBayesCollision")
        .def(py::init())
        .def_readwrite("link_name", &ContactBayesCollision::linkName)
        .def_readwrite("box_vertices_names", &ContactBayesCollision::boxVerticesNames)
        .def_readwrite("box_vertices", &ContactBayesCollision::boxVertices)
        .def_readwrite("nr_active_contacts", &ContactBayesCollision::nrActiveContacts)
        .def_readwrite("stable_contact", &ContactBayesCollision::stableContact)
        .def_readwrite("switch_time", &ContactBayesCollision::switchTime)
        .def_readwrite("position", &ContactBayesCollision::position)
        .def_readwrite("velocity", &ContactBayesCollision::velocity)
        .def_readwrite("vertices_positions_inertial", &ContactBayesCollision::verticesPositionsInertial)
        .def_readwrite("vertices_velocities_inertial", &ContactBayesCollision::verticesVelocitiesInertial)
        .def_readwrite("vertices_distances_to_nearest_plane", &ContactBayesCollision::verticesDistancesToNearestPlane)
        .def_readwrite("contact_probabilities", &ContactBayesCollision::contactProbabilities)
        .def_readwrite("swing_probabilities", &ContactBayesCollision::swingProbabilities)
        .def_readwrite("is_active", &ContactBayesCollision::isActive)
        .def_readwrite("vertices_switch_times", &ContactBayesCollision::verticesSwitchTimes);

    py::class_<ContactBayesManager, ContactDetector>(module, "ContactBayesManager")
        .def(py::init())
        .def(
            "initialize",
            [](ContactBayesManager& impl, std::shared_ptr<IParametersHandler> handler) -> bool {
                return impl.initialize(handler);
            },
            py::arg("handler"))
        .def("advance", &ContactBayesManager::advance)
        .def("reset_contacts", &ContactBayesManager::resetContacts)
        .def("get", py::overload_cast<>(&ContactBayesManager::get, py::const_))
        .def("get", py::overload_cast<const std::string&>(&ContactBayesManager::get, py::const_),
             py::arg("contact_name"))
        .def("is_valid", &ContactBayesManager::isValid)
        .def("set_kindyn", &ContactBayesManager::setKinDyn)
        .def("set_current_time", &ContactBayesManager::setCurrentTime)
        .def("prepare", &ContactBayesManager::prepare)
        .def("get_collision_data", &ContactBayesManager::getCollisionData)
        .def("get_lowest_height_frame_index", &ContactBayesManager::getLowestHeightFrameIndex)
        .def("get_lowest_height_frame_name", &ContactBayesManager::getLowestHeightFrameName)
        .def("set_robot_state_externally", &ContactBayesManager::setRobotStateExternally);

}

