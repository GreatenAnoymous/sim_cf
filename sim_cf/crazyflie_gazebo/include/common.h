#ifndef SITL_GAZEBO_COMMON_H_
#define SITL_GAZEBO_COMMON_H_
/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#include <Eigen/Dense>
#include <gazebo/gazebo.hh>
#include <ignition/math.hh>

namespace gazebo {

//===============================================================================================//
//========================================= DEBUGGING ===========================================//
//===============================================================================================//

/// \addtogroup Debug Print Switches
/// @{

// The following boolean constants enable/disable debug printing when certain plugin methods are called.
// Suitable for debugging purposes. Left on permanently can swamp std::out and can crash Gazebo.

static const bool kPrintOnPluginLoad    = false;
static const bool kPrintOnUpdates       = false;
static const bool kPrintOnMsgCallback   = false;

/// @}

// Default values
// static const std::string kDefaultNamespace = "";
// static constexpr double kDefaultRotorVelocitySlowdownSim = 10.0;

//===============================================================================================//
//================================== TOPICS FOR ROS INTERFACE ===================================//
//===============================================================================================//

// These should perhaps be defined in an .sdf/.xacro file instead?
static const std::string kConnectGazeboToRosSubtopic = "connect_gazebo_to_ros_subtopic";
static const std::string kConnectRosToGazeboSubtopic = "connect_ros_to_gazebo_subtopic";

/// \brief    Special-case topic for ROS interface plugin to listen to (if present)
///           and broadcast transforms to the ROS system.
static const std::string kBroadcastTransformSubtopic = "broadcast_transform";

/**
 * \brief Obtains a parameter from sdf.
 * \param[in] sdf Pointer to the sdf object.
 * \param[in] name Name of the parameter.
 * \param[out] param Param Variable to write the parameter to.
 * \param[in] default_value Default value, if the parameter not available.
 * \param[in] verbose If true, gzerror if the parameter is not available.
 */
template<class T>
bool getSdfParam(sdf::ElementPtr sdf, const std::string& name, T& param, const T& default_value, const bool& verbose =
                     false) {
  if (sdf->HasElement(name)) {
    param = sdf->GetElement(name)->Get<T>();
    return true;
  }
  else {
    param = default_value;
    if (verbose)
      gzerr << "[rotors_gazebo_plugins] Please specify a value for parameter \"" << name << "\".\n";
  }
  return false;
}

/**
 * \brief Get a math::Angle as an angle from [0, 360)
 */
double GetDegrees360(const ignition::math::Angle& angle) {
  double degrees = angle.Degree();
  while (degrees < 0.) degrees += 360.0;
  while (degrees >= 360.0) degrees -= 360.0;
  return degrees;
}


}  // namespace gazebo

template <typename T>
class FirstOrderFilter {
/*
This class can be used to apply a first order filter on a signal.
It allows different acceleration and deceleration time constants.

Short reveiw of discrete time implementation of firest order system:
Laplace:
    X(s)/U(s) = 1/(tau*s + 1)
continous time system:
    dx(t) = (-1/tau)*x(t) + (1/tau)*u(t)
discretized system (ZoH):
    x(k+1) = exp(samplingTime*(-1/tau))*x(k) + (1 - exp(samplingTime*(-1/tau))) * u(k)
*/

  public:
    FirstOrderFilter(double timeConstantUp, double timeConstantDown, T initialState):
      timeConstantUp_(timeConstantUp),
      timeConstantDown_(timeConstantDown),
      previousState_(initialState) {}

    T updateFilter(T inputState, double samplingTime) {
      /*
      This method will apply a first order filter on the inputState.
      */
      T outputState;
      if(inputState > previousState_){
        // Calcuate the outputState if accelerating.
        double alphaUp = exp(- samplingTime / timeConstantUp_);
        // x(k+1) = Ad*x(k) + Bd*u(k)
        outputState = alphaUp * previousState_ + (1 - alphaUp) * inputState;

      }else{
        // Calculate the outputState if decelerating.
        double alphaDown = exp(- samplingTime / timeConstantDown_);
        outputState = alphaDown * previousState_ + (1 - alphaDown) * inputState;
      }
      previousState_ = outputState;
      return outputState;

    }
    ~FirstOrderFilter() {}

  protected:
    double timeConstantUp_;
    double timeConstantDown_;
    T previousState_;
};



/// Computes a quaternion from the 3-element small angle approximation theta.
template<class Derived>
Eigen::Quaternion<typename Derived::Scalar> QuaternionFromSmallAngle(const Eigen::MatrixBase<Derived> & theta) {
  typedef typename Derived::Scalar Scalar;
  EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived);
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);
  const Scalar q_squared = theta.squaredNorm() / 4.0;

  if (q_squared < 1) {
    return Eigen::Quaternion<Scalar>(sqrt(1 - q_squared), theta[0] * 0.5, theta[1] * 0.5, theta[2] * 0.5);
  }
  else {
    const Scalar w = 1.0 / sqrt(1 + q_squared);
    const Scalar f = w * 0.5;
    return Eigen::Quaternion<Scalar>(w, theta[0] * f, theta[1] * f, theta[2] * f);
  }
}

template<class In, class Out>
void copyPosition(const In& in, Out* out) {
  out->x = in.x;
  out->y = in.y;
  out->z = in.z;
}

#if GAZEBO_MAJOR_VERSION < 9
inline ignition::math::Vector3d ignitionFromGazeboMath(const gazebo::math::Vector3 &vec_gz) {
  return ignition::math::Vector3d(vec_gz.x, vec_gz.y, vec_gz.z);
}

inline ignition::math::Pose3d ignitionFromGazeboMath(const gazebo::math::Pose &pose_gz) {

  return ignition::math::Pose3d(pose_gz.pos.x, pose_gz.pos.y, pose_gz.pos.z,
                                pose_gz.rot.w, pose_gz.rot.x, pose_gz.rot.y, pose_gz.rot.z);
}
#endif

#endif  // SITL_GAZEBO_COMMON_H_
