// Copyright 2020-2022 The MathWorks, Inc.
// Common copy functions for velocity_servo_tag_interfaces/StereoFeatures
#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4100)
#pragma warning(disable : 4265)
#pragma warning(disable : 4456)
#pragma warning(disable : 4458)
#pragma warning(disable : 4946)
#pragma warning(disable : 4244)
#else
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wunused-local-typedefs"
#pragma GCC diagnostic ignored "-Wredundant-decls"
#pragma GCC diagnostic ignored "-Wnon-virtual-dtor"
#pragma GCC diagnostic ignored "-Wdelete-non-virtual-dtor"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wunused-variable"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif //_MSC_VER
#include "rclcpp/rclcpp.hpp"
#include "velocity_servo_tag_interfaces/msg/stereo_features.hpp"
#include "visibility_control.h"
#include "class_loader/multi_library_class_loader.hpp"
#include "ROS2PubSubTemplates.hpp"
class VELOCITY_SERVO_TAG_INTERFACES_EXPORT ros2_velocity_servo_tag_interfaces_msg_StereoFeatures_common : public MATLABROS2MsgInterface<velocity_servo_tag_interfaces::msg::StereoFeatures> {
  public:
    virtual ~ros2_velocity_servo_tag_interfaces_msg_StereoFeatures_common(){}
    virtual void copy_from_struct(velocity_servo_tag_interfaces::msg::StereoFeatures* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const velocity_servo_tag_interfaces::msg::StereoFeatures* msg, MultiLibLoader loader, size_t size = 1);
};
  void ros2_velocity_servo_tag_interfaces_msg_StereoFeatures_common::copy_from_struct(velocity_servo_tag_interfaces::msg::StereoFeatures* msg, const matlab::data::Struct& arr,
               MultiLibLoader loader) {
    try {
        //left_sequence
        const matlab::data::TypedArray<uint32_t> left_sequence_arr = arr["left_sequence"];
        msg->left_sequence = left_sequence_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'left_sequence' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'left_sequence' is wrong type; expected a uint32.");
    }
    try {
        //right_sequence
        const matlab::data::TypedArray<uint32_t> right_sequence_arr = arr["right_sequence"];
        msg->right_sequence = right_sequence_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'right_sequence' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'right_sequence' is wrong type; expected a uint32.");
    }
    try {
        //left_capture_stamp
        const matlab::data::StructArray left_capture_stamp_arr = arr["left_capture_stamp"];
        auto msgClassPtr_left_capture_stamp = getCommonObject<builtin_interfaces::msg::Time>("ros2_builtin_interfaces_msg_Time_common",loader);
        msgClassPtr_left_capture_stamp->copy_from_struct(&msg->left_capture_stamp,left_capture_stamp_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'left_capture_stamp' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'left_capture_stamp' is wrong type; expected a struct.");
    }
    try {
        //right_capture_stamp
        const matlab::data::StructArray right_capture_stamp_arr = arr["right_capture_stamp"];
        auto msgClassPtr_right_capture_stamp = getCommonObject<builtin_interfaces::msg::Time>("ros2_builtin_interfaces_msg_Time_common",loader);
        msgClassPtr_right_capture_stamp->copy_from_struct(&msg->right_capture_stamp,right_capture_stamp_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'right_capture_stamp' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'right_capture_stamp' is wrong type; expected a struct.");
    }
    try {
        //valid_left
        const matlab::data::TypedArray<bool> valid_left_arr = arr["valid_left"];
        msg->valid_left = valid_left_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'valid_left' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'valid_left' is wrong type; expected a logical.");
    }
    try {
        //valid_right
        const matlab::data::TypedArray<bool> valid_right_arr = arr["valid_right"];
        msg->valid_right = valid_right_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'valid_right' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'valid_right' is wrong type; expected a logical.");
    }
    try {
        //u_left
        const matlab::data::TypedArray<double> u_left_arr = arr["u_left"];
        msg->u_left = u_left_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'u_left' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'u_left' is wrong type; expected a double.");
    }
    try {
        //v_left
        const matlab::data::TypedArray<double> v_left_arr = arr["v_left"];
        msg->v_left = v_left_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'v_left' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'v_left' is wrong type; expected a double.");
    }
    try {
        //u_right
        const matlab::data::TypedArray<double> u_right_arr = arr["u_right"];
        msg->u_right = u_right_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'u_right' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'u_right' is wrong type; expected a double.");
    }
    try {
        //v_right
        const matlab::data::TypedArray<double> v_right_arr = arr["v_right"];
        msg->v_right = v_right_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'v_right' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'v_right' is wrong type; expected a double.");
    }
    try {
        //scale_left
        const matlab::data::TypedArray<double> scale_left_arr = arr["scale_left"];
        msg->scale_left = scale_left_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'scale_left' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'scale_left' is wrong type; expected a double.");
    }
    try {
        //scale_right
        const matlab::data::TypedArray<double> scale_right_arr = arr["scale_right"];
        msg->scale_right = scale_right_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'scale_right' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'scale_right' is wrong type; expected a double.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T ros2_velocity_servo_tag_interfaces_msg_StereoFeatures_common::get_arr(MDFactory_T& factory, const velocity_servo_tag_interfaces::msg::StereoFeatures* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","left_sequence","right_sequence","left_capture_stamp","right_capture_stamp","valid_left","valid_right","u_left","v_left","u_right","v_right","scale_left","scale_right"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("velocity_servo_tag_interfaces/StereoFeatures");
    // left_sequence
    auto currentElement_left_sequence = (msg + ctr)->left_sequence;
    outArray[ctr]["left_sequence"] = factory.createScalar(currentElement_left_sequence);
    // right_sequence
    auto currentElement_right_sequence = (msg + ctr)->right_sequence;
    outArray[ctr]["right_sequence"] = factory.createScalar(currentElement_right_sequence);
    // left_capture_stamp
    auto currentElement_left_capture_stamp = (msg + ctr)->left_capture_stamp;
    auto msgClassPtr_left_capture_stamp = getCommonObject<builtin_interfaces::msg::Time>("ros2_builtin_interfaces_msg_Time_common",loader);
    outArray[ctr]["left_capture_stamp"] = msgClassPtr_left_capture_stamp->get_arr(factory, &currentElement_left_capture_stamp, loader);
    // right_capture_stamp
    auto currentElement_right_capture_stamp = (msg + ctr)->right_capture_stamp;
    auto msgClassPtr_right_capture_stamp = getCommonObject<builtin_interfaces::msg::Time>("ros2_builtin_interfaces_msg_Time_common",loader);
    outArray[ctr]["right_capture_stamp"] = msgClassPtr_right_capture_stamp->get_arr(factory, &currentElement_right_capture_stamp, loader);
    // valid_left
    auto currentElement_valid_left = (msg + ctr)->valid_left;
    outArray[ctr]["valid_left"] = factory.createScalar(currentElement_valid_left);
    // valid_right
    auto currentElement_valid_right = (msg + ctr)->valid_right;
    outArray[ctr]["valid_right"] = factory.createScalar(currentElement_valid_right);
    // u_left
    auto currentElement_u_left = (msg + ctr)->u_left;
    outArray[ctr]["u_left"] = factory.createScalar(currentElement_u_left);
    // v_left
    auto currentElement_v_left = (msg + ctr)->v_left;
    outArray[ctr]["v_left"] = factory.createScalar(currentElement_v_left);
    // u_right
    auto currentElement_u_right = (msg + ctr)->u_right;
    outArray[ctr]["u_right"] = factory.createScalar(currentElement_u_right);
    // v_right
    auto currentElement_v_right = (msg + ctr)->v_right;
    outArray[ctr]["v_right"] = factory.createScalar(currentElement_v_right);
    // scale_left
    auto currentElement_scale_left = (msg + ctr)->scale_left;
    outArray[ctr]["scale_left"] = factory.createScalar(currentElement_scale_left);
    // scale_right
    auto currentElement_scale_right = (msg + ctr)->scale_right;
    outArray[ctr]["scale_right"] = factory.createScalar(currentElement_scale_right);
    }
    return std::move(outArray);
  } 
class VELOCITY_SERVO_TAG_INTERFACES_EXPORT ros2_velocity_servo_tag_interfaces_StereoFeatures_message : public ROS2MsgElementInterfaceFactory {
  public:
    virtual ~ros2_velocity_servo_tag_interfaces_StereoFeatures_message(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType /*type*/);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType /*type*/);
    virtual std::shared_ptr<void> generateCppMessage(ElementType /*type*/, const matlab::data::StructArray& /* arr */, MultiLibLoader /* loader */, std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* /*commonObjMap*/);
    virtual matlab::data::StructArray generateMLMessage(ElementType  /*type*/ ,void*  /* msg */, MultiLibLoader /* loader */ , std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* /*commonObjMap*/);
};  
  std::shared_ptr<MATLABPublisherInterface> 
          ros2_velocity_servo_tag_interfaces_StereoFeatures_message::generatePublisherInterface(ElementType /*type*/){
    return std::make_shared<ROS2PublisherImpl<velocity_servo_tag_interfaces::msg::StereoFeatures,ros2_velocity_servo_tag_interfaces_msg_StereoFeatures_common>>();
  }
  std::shared_ptr<MATLABSubscriberInterface> 
         ros2_velocity_servo_tag_interfaces_StereoFeatures_message::generateSubscriberInterface(ElementType /*type*/){
    return std::make_shared<ROS2SubscriberImpl<velocity_servo_tag_interfaces::msg::StereoFeatures,ros2_velocity_servo_tag_interfaces_msg_StereoFeatures_common>>();
  }
  std::shared_ptr<void> ros2_velocity_servo_tag_interfaces_StereoFeatures_message::generateCppMessage(ElementType /*type*/, 
                                           const matlab::data::StructArray& arr,
                                           MultiLibLoader loader,
                                           std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* commonObjMap){
    auto msg = std::make_shared<velocity_servo_tag_interfaces::msg::StereoFeatures>();
    ros2_velocity_servo_tag_interfaces_msg_StereoFeatures_common commonObj;
    commonObj.mCommonObjMap = commonObjMap;
    commonObj.copy_from_struct(msg.get(), arr[0], loader);
    return msg;
  }
  matlab::data::StructArray ros2_velocity_servo_tag_interfaces_StereoFeatures_message::generateMLMessage(ElementType  /*type*/ ,
                                                    void*  msg ,
                                                    MultiLibLoader  loader ,
                                                    std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>*  commonObjMap ){
    ros2_velocity_servo_tag_interfaces_msg_StereoFeatures_common commonObj;	
    commonObj.mCommonObjMap = commonObjMap;	
    MDFactory_T factory;
    return commonObj.get_arr(factory, (velocity_servo_tag_interfaces::msg::StereoFeatures*)msg, loader);			
 }
#include "class_loader/register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(ros2_velocity_servo_tag_interfaces_msg_StereoFeatures_common, MATLABROS2MsgInterface<velocity_servo_tag_interfaces::msg::StereoFeatures>)
CLASS_LOADER_REGISTER_CLASS(ros2_velocity_servo_tag_interfaces_StereoFeatures_message, ROS2MsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER