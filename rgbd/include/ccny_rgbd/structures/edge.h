#ifndef CCNY_RGBD_EDGE_H
#define CCNY_RGBD_EDGE_H


#include "ccny_rgbd/util.h"

namespace ccny_rgbd
{

class Edge
{
  public:

    Edge();
    
    int index_a;
    int index_b;

    tf::Transform a2b;
    bool tf_computed;

    bool consecutive_association;

    bool ransac_association;
};

} //namespace ccny_rgbd


#endif // CCNY_RGBD_EDGE_H
