#include "image_undistort/image_undistort.h"
#include "image_undistort/camera_parameters.h"
#include "image_undistort/undistorter.h"

namespace image_undistort {

ImageUndistort::ImageUndistort(const ros::NodeHandle& nh,
                               const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      it_(nh_),
      undistorter_ptr_(nullptr),
      frame_counter_(0) {
    // set parameters from ros
    bool input_camera_info_from_ros_params;
    // read boolean parameter specifying if the input parameters have to read from ros params
    nh_private_.param("input_camera_info_from_ros_params",
                      input_camera_info_from_ros_params,
                      kDefaultInputCameraInfoFromROSParams);
    // read parameter specifying if radtan model has to be renamed as plumb bob model
    nh_private_.param("rename_radtan_plumb_bob", rename_radtan_plumb_bob_,
                      kDefaultRenameRadtanPlumbBob);
    // read parameter specifying how the output camera info should be set
    std::string output_camera_info_source_in;
    nh_private_.param("output_camera_info_source", output_camera_info_source_in,
                      kDefaultOutputCameraInfoSource);
    if (output_camera_info_source_in == "auto_generated") {
        output_camera_info_source_ = OutputInfoSource::AUTO_GENERATED;
    } else if (output_camera_info_source_in == "match_input") {
        output_camera_info_source_ = OutputInfoSource::MATCH_INPUT;
    } else if (output_camera_info_source_in == "ros_params") {
        output_camera_info_source_ = OutputInfoSource::ROS_PARAMS;
    } else if (output_camera_info_source_in == "camera_info") {
        output_camera_info_source_ = OutputInfoSource::CAMERA_INFO;
    } else {
        ROS_ERROR(
                    "Invalid camera source given, valid options are auto_generated, "
                    "match_input, ros_params and camera_info. Defaulting to "
                    "auto_generated");
        output_camera_info_source_ = OutputInfoSource::AUTO_GENERATED;
    }
    // read parameter specifying the queue_size
    nh_private_.param("queue_size", queue_size_, kQueueSize);
    if (queue_size_ < 1) {
        ROS_ERROR("Queue size must be >= 1, setting to 1");
        queue_size_ = 1;
    }

    nh_private_.param("process_image", process_image_, kDefaultProcessImage);
    if (!process_image_ && !input_camera_info_from_ros_params) {
        ROS_FATAL(
                    "Settings specify no image processing and not to generate camera info "
                    "from file. This leaves nothing for the node to do, exiting");
        ros::shutdown();
        exit(EXIT_SUCCESS);
    }
    // read parameter specifying the scale of the output image
    nh_private_.param("scale", scale_, kDefaultScale);

    // read boolean flag specifying if undistortion has to be done or not
    bool undistort_image;
    nh_private_.param("undistort_image", undistort_image, kDefaultUndistortImage);
    DistortionProcessing distortion_processing;
    if (undistort_image) {
        distortion_processing == DistortionProcessing::UNDISTORT;
    } else {
        distortion_processing == DistortionProcessing::PRESERVE;
    }
    camera_parameters_pair_ptr_ =
            std::make_shared<CameraParametersPair>(distortion_processing);

    // parameter specifying how many frame to skip
    nh_private_.param("process_every_nth_frame", process_every_nth_frame_,
                      kDefaultProcessEveryNthFrame);
    // paramters specifying the type of output image (check cv_bridge)
    nh_private_.param("output_image_type", output_image_type_,
                      kDefaultOutputImageType);
    // check output type string is correctly formatted
    if (!output_image_type_.empty()) {
        try {
            cv_bridge::getCvType(output_image_type_);
        } catch (const cv_bridge::Exception& e) {
            ROS_ERROR_STREAM(
                        "cv_bridge error while setting output_image_type, output will match "
                        "input type. "
                        << e.what());
            output_image_type_ = "";
        }
    }

    // read boolean parameter specifying if tf between input and output frames has to be published
    nh_private_.param("publish_tf", publish_tf_, kDefaultPublishTF);
    // name of the output image frame
    nh_private_.param("output_frame", output_frame_, kDefaultOutputFrame);
    if (output_frame_.empty()) {
        ROS_ERROR("Output frame cannot be blank, setting to default");
        output_frame_ = kDefaultOutputFrame;
    }
    // read boolean parameter specifying if input frame has to be renamed
    nh_private_.param("rename_input_frame", rename_input_frame_,
                      kDefaultRenameInputFrame);
    // read parameter specifying the new name of the input frame if it has to be renamed
    nh_private_.param("input_frame", input_frame_, kDefaultInputFrame);
    if (input_frame_.empty()) {
        ROS_ERROR("Input frame cannot be blank, setting to default");
        input_frame_ = kDefaultInputFrame;
    }

    // setup subscribers
    std::string input_camera_namespace;
    if (input_camera_info_from_ros_params) {
        // read namespace of the input camera
        nh_private_.param("input_camera_namespace", input_camera_namespace,
                          kDefaultInputCameraNamespace);
        // load the camera parameters
        if (!camera_parameters_pair_ptr_->setCameraParameters(
                    nh_private_, input_camera_namespace, CameraIO::INPUT)) {
            ROS_FATAL("Loading of input camera parameters failed, exiting");
            ros::shutdown();
            exit(EXIT_FAILURE);
        }
        // subscribe to the image topic anf=d assign callback to the imageCallback function
        image_sub_ = it_.subscribe("input/image", queue_size_,
                                   &ImageUndistort::imageCallback, this);
    } else {
        // subscribe to the image topic and assign callback to the cameraCallback function
        camera_sub_ = it_.subscribeCamera("input/image", queue_size_,
                                          &ImageUndistort::cameraCallback, this);
    }

    // setup publishers
    if (process_image_) {
        bool pub_camera_info_output = true;
        if (output_camera_info_source_ == OutputInfoSource::ROS_PARAMS) {
            std::string output_camera_namespace;
            // read namespace of the output camera
            nh_private_.param("output_camera_namespace", output_camera_namespace,
                              kDefaultOutputCameraNamespace);
            // set parameters of the output camera
            if (!camera_parameters_pair_ptr_->setCameraParameters(
                        nh_private_, output_camera_namespace, CameraIO::OUTPUT)) {
                ROS_FATAL("Loading of output camera parameters failed, exiting");
                ros::shutdown();
                exit(EXIT_FAILURE);
            }
        } else if (output_camera_info_source_ == OutputInfoSource::MATCH_INPUT) {
            camera_parameters_pair_ptr_->setOutputFromInput();
        } else if (output_camera_info_source_ == OutputInfoSource::AUTO_GENERATED) {
            camera_parameters_pair_ptr_->setOptimalOutputCameraParameters(scale_);
        } else {
            camera_info_sub_ =
                    nh_.subscribe("output/camera_info", queue_size_,
                                  &ImageUndistort::cameraInfoCallback, this);
            pub_camera_info_output = false;
        }

        if (pub_camera_info_output) {
            camera_pub_ = it_.advertiseCamera("output/image", queue_size_);
        } else {
            image_pub_ = it_.advertise("output/image", queue_size_);
        }
    } else {
        camera_parameters_pair_ptr_->setOutputFromInput();

        camera_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>(
                    "output/camera_info", queue_size_);
    }
}

void ImageUndistort::imageCallback(const sensor_msgs::ImageConstPtr& image_msg_in)
{
    if (++frame_counter_ < process_every_nth_frame_) {
        return;
    }
    frame_counter_ = 0;

    if (!process_image_)
    {// if image should not be processed, just copy the camera info, publish it and return
        sensor_msgs::CameraInfo camera_info;
        camera_info.header = image_msg_in->header;
        if (rename_input_frame_) {
            camera_info.header.frame_id = input_frame_;
        }
        camera_parameters_pair_ptr_->generateCameraInfoMessage(CameraIO::OUTPUT,
                                                               &camera_info);
        if (rename_radtan_plumb_bob_ && camera_info.distortion_model == "radtan") {
            camera_info.distortion_model = "plumb_bob";
        }
        camera_info_pub_.publish(camera_info);
        return;
    }
    // create a pointer ot the image message
    cv_bridge::CvImageConstPtr image_in_ptr =
            cv_bridge::toCvShare(image_msg_in, output_image_type_);

    std::string encoding = image_in_ptr->encoding;
    if (encoding == "8UC1") {
        // ros does not recognize 8UC1 and using it will crash anything that does a
        // color conversion
        encoding = "mono8";
    }
    // create pointer to a new cv bridge image
    cv_bridge::CvImagePtr image_out_ptr(
                new cv_bridge::CvImage(image_in_ptr->header, encoding));

    // if undistorter not built or built using old data update it
    if (!undistorter_ptr_ || (undistorter_ptr_->getCameraParametersPair() !=
                              *camera_parameters_pair_ptr_)) {
        try {
            undistorter_ptr_ =
                    std::make_shared<Undistorter>(*camera_parameters_pair_ptr_);
        } catch (std::runtime_error e) {
            ROS_ERROR("%s", e.what());
            return;
        }
    }

    // undistort the image
    undistorter_ptr_->undistortImage(image_in_ptr->image,
                                     &(image_out_ptr->image));
    // add header to the output image
    image_out_ptr->header.frame_id = output_frame_;

    // if camera info was just read in from a topic don't republish it
    if (output_camera_info_source_ == OutputInfoSource::CAMERA_INFO) {
        image_pub_.publish(*(image_out_ptr->toImageMsg()));
    } else {
        sensor_msgs::CameraInfo camera_info;
        camera_info.header = image_out_ptr->header;
        if (rename_input_frame_) {
            camera_info.header.frame_id = input_frame_;
        }
        camera_parameters_pair_ptr_->generateCameraInfoMessage(CameraIO::OUTPUT,
                                                               &camera_info);
        if (rename_radtan_plumb_bob_ && camera_info.distortion_model == "radtan") {
            camera_info.distortion_model = "plumb_bob";
        }
        camera_pub_.publish(*(image_out_ptr->toImageMsg()), camera_info);
    }

    if (publish_tf_) {
        Eigen::Matrix4d T =
                camera_parameters_pair_ptr_->getInputPtr()->T().inverse() *
                camera_parameters_pair_ptr_->getOutputPtr()->T();

        tf::Matrix3x3 R_ros;
        tf::Vector3 p_ros;
        tf::matrixEigenToTF(T.topLeftCorner<3, 3>(), R_ros);
        tf::vectorEigenToTF(T.topRightCorner<3, 1>(), p_ros);
        tf::Transform(R_ros, p_ros);

        std::string frame = image_in_ptr->header.frame_id;
        if (rename_input_frame_) {
            frame = input_frame_;
        }
        if (frame.empty()) {
            ROS_ERROR_ONCE("Image frame name is blank, cannot construct tf");
        } else {
            br_.sendTransform(tf::StampedTransform(tf::Transform(R_ros, p_ros),
                                                   image_out_ptr->header.stamp, frame,
                                                   output_frame_));
        }
    }
}

void ImageUndistort::cameraCallback(
        const sensor_msgs::ImageConstPtr& image_msg,
        const sensor_msgs::CameraInfoConstPtr& camera_info) {
    camera_parameters_pair_ptr_->setCameraParameters(*camera_info,
                                                     CameraIO::INPUT);
    if (output_camera_info_source_ == OutputInfoSource::MATCH_INPUT) {
        camera_parameters_pair_ptr_->setOutputFromInput();
    } else if (output_camera_info_source_ == OutputInfoSource::AUTO_GENERATED) {
        camera_parameters_pair_ptr_->setOptimalOutputCameraParameters(scale_);
    }

    imageCallback(image_msg);
}

void ImageUndistort::cameraInfoCallback(
        const sensor_msgs::CameraInfoConstPtr& camera_info) {
    if (!camera_parameters_pair_ptr_->setCameraParameters(*camera_info,
                                                          CameraIO::OUTPUT)) {
        ROS_ERROR("Setting output camera from ros message failed");
    }
}
}
