#ifndef CCNY_RGBD_RESIZER_NODELET_H
#define CCNY_RGBD_RESIZER_NODELET_H

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "ccny_rgbd/filter/resizer.h"

namespace ccny_rgbd
{
  class ResizerNodelet : public nodelet::Nodelet
  {
    public:
      virtual void onInit ();

    private:
      ccny_rgbd::Resizer * r_;  // FIXME: change to smart pointer
  };
}

#endif // CCNY_RGBD_RESIZER_NODELET_H
