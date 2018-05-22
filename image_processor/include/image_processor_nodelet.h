/*
 * COPYRIGHT AND PERMISSION NOTICE
 * Penn Software MSCKF_VIO
 * Copyright (C) 2017 The Trustees of the University of Pennsylvania
 * All rights reserved.
 */

#ifndef IMAGE_PROCESSOR_NODELET_H
#define IMAGE_PROCESSOR_NODELET_H

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <image_processor.h>

class ImageProcessorNodelet : public nodelet::Nodelet {
public:
  ImageProcessorNodelet() { return; }
  ~ImageProcessorNodelet() { return; }

private:
  virtual void onInit();
  ImageProcessorPtr img_processor_ptr;
};

#endif

