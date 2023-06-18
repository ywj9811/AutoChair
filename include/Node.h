#ifndef NODE_H
#define NODE_H

#include "common_header.h"
#include "opencv2/opencv.hpp"

class Node 
{
  public:
    int pre_node_idx;
    float f, g, h;
    cv::Point position;

  public:
    Node ( cv::Point _position ) 
    {
        pre_node_idx = -1;
        position = _position;
        f = 0;
        g = 0;
        h = 0;
    }

    Node (int node_idx, cv::Point _position ) 
    {
        pre_node_idx = node_idx;
        position = _position;
        f = 0;
        g = 0;
        h = 0;
    }
};

#endif
