/* BSD 3-Clause License

Copyright (c) 2025, NXROBO

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

#ifndef SPARK_APP_SPARK_FOLLOWER_SRC_NXFOLLOWER_NODELET_HPP_
#define SPARK_APP_SPARK_FOLLOWER_SRC_NXFOLLOWER_NODELET_HPP_

#include <nodelet/nodelet.h>

#include "nxfollower.hpp"

namespace nxfollower
{
class NxFollowerNodelet : public nodelet::Nodelet
{
protected:
  NxFollowerNode *nxfollower_node;
  ros::NodeHandle nh, pnh;
  virtual void onInit()
  {
    ROS_INFO("Start to bring up nxfollower!");
    nh = getNodeHandle();
    pnh = getPrivateNodeHandle();
    nxfollower_node = new NxFollowerNode(nh, pnh);
  }

public:
  ~NxFollowerNodelet()
  {
    if (nxfollower_node != NULL)
    {
      delete nxfollower_node;
      nxfollower_node = NULL;
    }
  }
};
}

// watch the capitalization carefully
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(nxfollower::NxFollowerNodelet, nodelet::Nodelet)

#endif
