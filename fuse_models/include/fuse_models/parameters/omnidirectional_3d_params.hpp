/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2024, Giacomo Franchini
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef FUSE_MODELS__PARAMETERS__OMNIDIRECTIONAL_3D_PARAMS_HPP_
#define FUSE_MODELS__PARAMETERS__OMNIDIRECTIONAL_3D_PARAMS_HPP_

#include <string>
#include <vector>

#include <fuse_models/parameters/parameter_base.hpp>

#include <fuse_core/loss.hpp>
#include <fuse_core/parameter.hpp>
#include <fuse_core/eigen.hpp>
#include <fuse_variables/orientation_3d_stamped.hpp>
#include <fuse_variables/position_3d_stamped.hpp>
#include <fuse_variables/velocity_angular_3d_stamped.hpp>
#include <fuse_variables/velocity_linear_3d_stamped.hpp>


namespace fuse_models
{

namespace parameters
{

/**
 * @brief Defines the set of parameters required by the Omnidirectional3D class
 */
struct Omnidirectional3DParams : public ParameterBase
{
public:
  /**
   * @brief Method for loading parameter values from ROS.
   *
   * @param[in] interfaces - The node interfaces with which to load parameters
   * @param[in] ns - The parameter namespace to use
   */
  void loadFromROS(
    fuse_core::node_interfaces::NodeInterfaces<
      fuse_core::node_interfaces::Base,
      fuse_core::node_interfaces::Logging,
      fuse_core::node_interfaces::Parameters
    > interfaces,
    const std::string & ns)
  {
    scale_process_noise = fuse_core::getParam(
      interfaces, fuse_core::joinParameterName(
        ns,
        "scale_process_noise"),
      scale_process_noise);

    disable_checks = fuse_core::getParam(
      interfaces, fuse_core::joinParameterName(
        ns,
        "disable_checks"),
      disable_checks);

    fuse_core::getPositiveParam(
      interfaces, fuse_core::joinParameterName(
        ns,
        "velocity_linear_norm_min"),
      velocity_linear_norm_min);

    fuse_core::getPositiveParam(
      interfaces, fuse_core::joinParameterName(
        ns,
        "velocity_angular_norm_min"),
      velocity_angular_norm_min);

    fuse_core::getPositiveParam(
      interfaces, fuse_core::joinParameterName(
        ns,
        "buffer_length"),
      buffer_length);

    process_noise_covariance =
      fuse_core::getCovarianceDiagonalParam<15>(
      interfaces,
      fuse_core::joinParameterName(ns, "process_noise_diagonal"), 0.001);
  }

  bool scale_process_noise {false}; //!< Whether to scale the process noise by the time delta
  bool disable_checks {false}; //!< Whether to disable checks for valid input data
  double velocity_linear_norm_min {1e-3}; //!< The minimum linear velocity norm to consider valid
  double velocity_angular_norm_min {1e-3}; //!< The minimum angular velocity norm to consider valid
  fuse_core::Matrix15d process_noise_covariance; //!< The diagonal of the process noise covariance matrix
  double buffer_length {3.0}; //!< The length of the state history buffer
};

}  // namespace parameters

}  // namespace fuse_models

#endif  // FUSE_MODELS__PARAMETERS__OMNIDIRECTIONAL_3D_PARAMS_HPP_
