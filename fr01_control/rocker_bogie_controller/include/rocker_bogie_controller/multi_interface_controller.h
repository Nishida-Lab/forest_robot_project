///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2015, PAL Robotics S.L.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the names of PAL Robotics S.L. nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////

/** \author Adolfo Rodríguez Tsouroukdissian */

#ifndef CONTROLLER_INTERFACE_MULTI_INTERFACE_CONTROLLER_H
#define CONTROLLER_INTERFACE_MULTI_INTERFACE_CONTROLLER_H

#include <algorithm>
#include <sstream>

#include <controller_interface/controller_base.h>
#include <hardware_interface/internal/demangle_symbol.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/hardware_interface.h>
#include <ros/ros.h>

namespace controller_interface
{

/** \cond HIDDEN_SYMBOLS */
namespace internal
{

template <class T>
bool hasInterface(hardware_interface::RobotHW* robot_hw);

template <class T>
void clearClaims(hardware_interface::RobotHW* robot_hw);

template <class T>
void extractInterfaceResources(hardware_interface::RobotHW* robot_hw_in,
                               hardware_interface::RobotHW* robot_hw_out);

template <class T>
void populateClaimedResources(hardware_interface::RobotHW*      robot_hw,
                              std::set<std::string>& claimed_resources);

template <class T>
std::string enumerateElements(const T& val,
                              const std::string& delimiter = ", ",
                              const std::string& prefix = "",
                              const std::string& suffix = "");

} // namespace
/** \endcond */


/**
 * \brief %Controller able to claim resources from multiple hardware interfaces.
 *
 * This particular controller implementation allows to claim resources from one
 * up to four different hardware interfaces. The types of these hardware
 * interfaces are specified as template parameters.
 *
 * An example multi-interface controller could claim, for instance, resources
 * from a position-controlled arm and velocity-controlled wheels. Another
 * example would be a controller claiming both position and effort interfaces
 * for the same robot resources, but this would require a robot with a custom
 * (non-exclusive) resource handling policy.
 *
 * By default, all specified hardware interfaces are required, and their
 * existence will be enforced by \ref initRequest. It is possible to make hardware
 * interfaces optional by means of the \c allow_optional_interfaces
 * \ref MultiInterfaceController::MultiInterfaceController "constructor" parameter.
 * This allows to write controllers where some interfaces are mandatory, and
 * others, if present, improve controller performance, but whose absence does not
 * prevent the controller from running.
 *
 * The following is an example of a controller claiming resources from velocity-
 * and effort-controlled joints.
 *
 * \code
 * #include <controller_interface/multi_interface_controller.h>
 * #include <hardware_interface/joint_command_interface.h>
 *
 * using namespace hardawre_interface;
 * class VelEffController : public
 *       controller_interface::MultiInterfaceController<VelocityJointInterface,
 *                                                      EffortJointInterface>
 * {
 * public:
 *   VelEffController() {}
 *
 *   bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle &n)
 *   {
 *     // robot_hw pointer only contains the two interfaces requested by the
 *     // controller. It is a subset of the entire robot, which may have more
 *     // hardware interfaces
 *
 *     // v and e below are guarranteed to be valid
 *     VelocityJointInterface* v = robot_hw->get<VelocityJointInterface>;
 *     EffortJointInterface*   e = robot_hw->get<EffortJointInterface>;
 *
 *     // Fetch resources from interfaces, perform rest of initialization
 *     //...
 *
 *     return true;
 *   }
 *   void starting(const ros::Time& time);
 *   void update(const ros::Time& time, const ros::Duration& period);
 *   void stopping(const ros::Time& time);
 * };
 * \endcode
 *
 * The following fragment is a modified version of the above example, where
 * controller interfaces are not required. It is left to the controller
 * implementer to verify interface validity. Only the initialization code is
 * shown.
 *
 * \code
 * class VelEffController : public
 *       controller_interface::MultiInterfaceController<VelocityJointInterface,
 *                                                      EffortJointInterface>
 * {
 * public:
 *   // Note true flag passed to parent class, allowing requested hardware
 *   // interfaces to be optional
 *   VelEffController()
 *    : controller_interface::MultiInterfaceController<VelocityJointInterface,
 *                                                     EffortJointInterface> (true)
 *   {}
 *
 *   bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle &n)
 *   {
 *     // robot_hw pointer contains at most the two interfaces requested by the
 *     // controller. It may have none, only one or both, depending on whether the
 *     // robot exposes them
 *
 *     // v is a required interface
 *     VelocityJointInterface* v = robot_hw->get<VelocityJointInterface>;
 *     if (!v)
 *     {
 *       return false;
 *     }
 *
 *     // e is an optional interface. If present, additional features are enabled.
 *     // Controller can still function if interface or some of its resources are
 *     // absent
 *     EffortJointInterface* e = robot_hw->get<EffortJointInterface>;
 *
 *     // Fetch resources from interfaces, perform rest of initialization
 *     //...
 *
 *     return true;
 *   }
 *   ...
 * };
 * \endcode
 *
 * \tparam T1 Hardware interface type.
 * This parameter is \e required.
 *
 * \tparam T2 Hardware interface type.
 * This parameter is \e optional. Leave unspecified if controller only claims
 * resources from a \e single hardware interface.
 *
 * \tparam T3 Hardware interface type.
 * This parameter is \e optional. Leave unspecified if controller only claims
 * resources from \e two hardware interfaces.
 *
 * \tparam T4 Hardware interface type.
 * This parameter is \e optional. Leave unspecified if controller only claims
 * resources from \e three hardware interfaces.
 *
 * \pre When specified, template parameters \c T1 to \c T4 must be different
 * types.
 */
template <class T1, class T2 = void, class T3 = void, class T4 = void>
class MultiInterfaceController: public ControllerBase
{
public:
  /**
   * \param allow_optional_interfaces If set to true, \ref initRequest will
   * not fail if one or more of the requested interfaces is not present.
   * If set to false (the default), all requested interfaces are required.
   */
  MultiInterfaceController(bool allow_optional_interfaces = false)
    : allow_optional_interfaces_(allow_optional_interfaces)
  {state_ = CONSTRUCTED;}

  virtual ~MultiInterfaceController() {}

  /** \name Non Real-Time Safe Functions
   *\{*/

  /**
   * \brief Custom controller initialization logic.
   *
   * In this method resources from different interfaces are claimed, and other
   * non real-time initialization is performed, such as setup of ROS interfaces
   * and resource pre-allocation.
   *
   * \param robot_hw Robot hardware abstraction containing a subset of the entire
   * robot. If \ref MultiInterfaceController::MultiInterfaceController
   * "MultiInterfaceController" was called with \c allow_optional_interfaces set
   * to \c false (the default), this parameter contains all the interfaces
   * requested by the controller.
   * If \c allow_optional_interfaces was set to \c false, this parameter may
   * contain none, some or all interfaces requested by the controller, depending
   * on whether the robot exposes them. Please refer to the code examples in the
   * \ref MultiInterfaceController "class description".
   *
   * \param controller_nh A NodeHandle in the namespace from which the controller
   * should read its configuration, and where it should set up its ROS
   * interface.
   *
   * \returns True if initialization was successful and the controller
   * is ready to be started.
   */
  virtual bool init(hardware_interface::RobotHW* /*robot_hw*/,
                    ros::NodeHandle&             /*controller_nh*/)
  {return true;}

  /**
   * \brief Custom controller initialization logic.
   *
   * In this method resources from different interfaces are claimed, and other
   * non real-time initialization is performed, such as setup of ROS interfaces
   * and resource pre-allocation.
   *
   * \param robot_hw Robot hardware abstraction containing a subset of the entire
   * robot. If \ref MultiInterfaceController::MultiInterfaceController
   * "MultiInterfaceController" was called with \c allow_optional_interfaces set
   * to \c false (the default), this parameter contains all the interfaces
   * requested by the controller.
   * If \c allow_optional_interfaces was set to \c false, this parameter may
   * contain none, some or all interfaces requested by the controller, depending
   * on whether the robot exposes them. Please refer to the code examples in the
   * \ref MultiInterfaceController "class description".
   *
   * \param root_nh A NodeHandle in the root of the controller manager namespace.
   * This is where the ROS interfaces are setup (publishers, subscribers, services).
   *
   * \param controller_nh A NodeHandle in the namespace of the controller.
   * This is where the controller-specific configuration resides.
   *
   * \returns True if initialization was successful and the controller
   * is ready to be started.
   */
  virtual bool init(hardware_interface::RobotHW* /*robot_hw*/,
                    ros::NodeHandle&             /*root_nh*/,
                    ros::NodeHandle&             /*controller_nh*/)
  {return true;}

protected:
  /**
   * \brief Initialize the controller from a RobotHW pointer.
   *
   * This calls \ref init with a RobotHW that is a subset of the input
   * \c robot_hw parameter, containing only the requested hardware interfaces
   * (all or some, depending on the value of \c allow_optional_interfaces passed
   * to the constructor).
   *
   * \param robot_hw The robot hardware abstraction.
   *
   * \param root_nh A NodeHandle in the root of the controller manager namespace.
   * This is where the ROS interfaces are setup (publishers, subscribers, services).
   *
   * \param controller_nh A NodeHandle in the namespace of the controller.
   * This is where the controller-specific configuration resides.
   *
   * \param[out] claimed_resources The resources claimed by this controller.
   * They can belong to multiple hardware interfaces.
   *
   * \returns True if initialization was successful and the controller
   * is ready to be started.
   */
  virtual bool initRequest(hardware_interface::RobotHW* robot_hw,
                           ros::NodeHandle&             root_nh,
                           ros::NodeHandle&             controller_nh,
                           std::set<std::string>&       claimed_resources)
  {
    // check if construction finished cleanly
    if (state_ != CONSTRUCTED){
      ROS_ERROR("Cannot initialize this controller because it failed to be constructed");
      return false;
    }

    // check for required hardware interfaces
    if (!allow_optional_interfaces_ && !hasRequiredInterfaces(robot_hw)) {return false;}

    // populate robot hardware abstraction containing only controller hardware interfaces (subset of robot)
    hardware_interface::RobotHW* robot_hw_ctrl_p = &robot_hw_ctrl_;
    extractInterfaceResources(robot_hw, robot_hw_ctrl_p);

    // custom controller initialization
    clearClaims(robot_hw_ctrl_p); // claims will be populated on controller init
    if (!init(robot_hw_ctrl_p, controller_nh) || !init(robot_hw_ctrl_p, root_nh, controller_nh))
    {
      ROS_ERROR("Failed to initialize the controller");
      return false;
    }

    // populate claimed resources
    claimed_resources.clear();
    populateClaimedResources(robot_hw_ctrl_p, claimed_resources);
    clearClaims(robot_hw_ctrl_p);
    // NOTE: Above, claims are cleared since we only want to know what they are and report them back
    // as an output parameter. Actual resource claiming by the controller is done when the controller
    // is start()ed

    // initialization successful
    state_ = INITIALIZED;
    return true;
  }

  /*\}*/

  virtual std::string getHardwareInterfaceType() const
  {
    return hardware_interface::internal::demangledTypeName<T1>();
  }

  /**
   * \brief Check if robot hardware abstraction contains all required interfaces.
   * \param robot_hw Robot hardware abstraction.
   * \return true if all required hardware interfaces are exposed by \c robot_hw,
   * false otherwise.
   */
  static bool hasRequiredInterfaces(hardware_interface::RobotHW* robot_hw)
  {
    using internal::hasInterface;
    return hasInterface<T1>(robot_hw) &&
           hasInterface<T2>(robot_hw) &&
           hasInterface<T3>(robot_hw) &&
           hasInterface<T4>(robot_hw);
  }

  /**
   * \brief Clear claims from all hardware interfaces requested by this controller.
   * \param robot_hw Robot hardware abstraction containing the interfaces whose
   * claims will be cleared.
   */
  static void clearClaims(hardware_interface::RobotHW* robot_hw)
  {
    using internal::clearClaims;
    clearClaims<T1>(robot_hw);
    clearClaims<T2>(robot_hw);
    clearClaims<T3>(robot_hw);
    clearClaims<T4>(robot_hw);
  }

  /**
   * \brief Extract all hardware interfaces requested by this controller from
   *  \c robot_hw_in, and add them also to \c robot_hw_out.
   * \param[in] robot_hw_in Robot hardware abstraction containing the interfaces
   * requested by this controller, and potentially others.
   * \param[out] robot_hw_out Robot hardware abstraction containing \e only the
   * interfaces requested by this controller.
   */
  static void extractInterfaceResources(hardware_interface::RobotHW* robot_hw_in,
                                        hardware_interface::RobotHW* robot_hw_out)
  {
    using internal::extractInterfaceResources;
    extractInterfaceResources<T1>(robot_hw_in, robot_hw_out);
    extractInterfaceResources<T2>(robot_hw_in, robot_hw_out);
    extractInterfaceResources<T3>(robot_hw_in, robot_hw_out);
    extractInterfaceResources<T4>(robot_hw_in, robot_hw_out);
  }

  /**
   * \brief Extract all hardware interfaces requested by this controller from
   *  \c robot_hw_in, and add them also to \c robot_hw_out.
   * \param[in] robot_hw_in Robot hardware abstraction containing the interfaces
   * requested by this controller, and potentially others.
   * \param[out] claimed_resources The resources claimed by this controller.
   * They can belong to multiple hardware interfaces.
   */
  static void populateClaimedResources(hardware_interface::RobotHW* robot_hw,
                                       std::set<std::string>&       claimed_resources)
  {
    using internal::populateClaimedResources;
    populateClaimedResources<T1>(robot_hw, claimed_resources);
    populateClaimedResources<T2>(robot_hw, claimed_resources);
    populateClaimedResources<T3>(robot_hw, claimed_resources);
    populateClaimedResources<T4>(robot_hw, claimed_resources);
  }

  /** Robot hardware abstraction containing only the subset of interfaces requested by the controller. */
  hardware_interface::RobotHW robot_hw_ctrl_;

  /** Flag to indicate if hardware interfaces are considered optional (i.e. non-required). */
  bool allow_optional_interfaces_;

private:
  MultiInterfaceController(const MultiInterfaceController& c);
  MultiInterfaceController& operator =(const MultiInterfaceController& c);
};


namespace internal
{

template <class T>
inline bool hasInterface(hardware_interface::RobotHW* robot_hw)
{
  T* hw = robot_hw->get<T>();
  if (!hw)
  {
    const std::string hw_name = hardware_interface::internal::demangledTypeName<T>();
    ROS_ERROR_STREAM("This controller requires a hardware interface of type '" << hw_name << "', " <<
                     "but is not exposed by the robot.");// Available interfaces in robot:\n" <<
                     //enumerateElements(robot_hw->getNames(), "\n", "- '", "'")); // delimiter, prefix, suffux
    return false;
  }
  return true;
}

// Specialization for unused template parameters defaulting to void
template <>
inline bool hasInterface<void>(hardware_interface::RobotHW* /*robot_hw*/) {return true;}

template <class T>
void clearClaims(hardware_interface::RobotHW* robot_hw)
{
  T* hw = robot_hw->get<T>();
  if (hw) {hw->clearClaims();}
}

// Specialization for unused template parameters defaulting to void
template <>
inline void clearClaims<void>(hardware_interface::RobotHW* /*robot_hw*/) {}

template <class T>
inline void extractInterfaceResources(hardware_interface::RobotHW* robot_hw_in,
                                      hardware_interface::RobotHW* robot_hw_out)
{
  T* hw = robot_hw_in->get<T>();
  if (hw) {robot_hw_out->registerInterface(hw);}
}

// Specialization for unused template parameters defaulting to void
template <>
inline void extractInterfaceResources<void>(hardware_interface::RobotHW* /*robot_hw_in*/,
                                            hardware_interface::RobotHW* /*robot_hw_out*/) {}

template <class T>
inline void populateClaimedResources(hardware_interface::RobotHW*      robot_hw,
                                     std::set<std::string>& claimed_resources)
{
  T* hw = robot_hw->get<T>();
  if (hw)
  {
    //hardware_interface::InterfaceResources iface_res;
    //iface_res.hardware_interface = hardware_interface::internal::demangledTypeName<T>();
    //iface_res.resources = hw->getClaims();
    //claimed_resources.push_back(iface_res);

    // TODO: must be fixed so that all claims to be registered
    claimed_resources = hw->getClaims();
  }
}

// Specialization for unused template parameters defaulting to void
template <>
inline void populateClaimedResources<void>(hardware_interface::RobotHW*      /*robot_hw*/,
                                            std::set<std::string>& /*claimed_resources*/) {}

template <class T>
inline std::string enumerateElements(const T&           val,
                                     const std::string& delimiter,
                                     const std::string& prefix,
                                     const std::string& suffix)
{
  std::string ret;
  if (val.empty()) {return ret;}

  const std::string sdp = suffix+delimiter+prefix;
  std::stringstream ss;
  ss << prefix;
  std::copy(val.begin(), val.end(), std::ostream_iterator<typename T::value_type>(ss, sdp.c_str()));
  ret = ss.str();
  if (!ret.empty()) {ret.erase(ret.size() - delimiter.size() - prefix.size());}
  return ret;
}

} // namespace

} // namespace

#endif
