/*
 * COPYRIGHT AND PERMISSION NOTICE
 * Penn Software MSCKF_VIO
 * Copyright (C) 2017 The Trustees of the University of Pennsylvania
 * All rights reserved.
 */

#include <image_processor_nodelet.h>

void ImageProcessorNodelet::onInit() {
  img_processor_ptr.reset(new ImageProcessor(getPrivateNodeHandle()));
  if (!img_processor_ptr->initialize()) {
    ROS_ERROR("Cannot initialize Image Processor...");
    return;
  }
  return;
}

PLUGINLIB_DECLARE_CLASS(ImageProcessor, ImageProcessorNodelet,
    ImageProcessorNodelet, nodelet::Nodelet);

