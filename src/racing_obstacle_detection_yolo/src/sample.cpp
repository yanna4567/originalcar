// Copyright (c) 2022，Horizon Robotics.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cv_bridge/cv_bridge.h>
#include <fstream>

#include "ai_msgs/msg/perception_targets.hpp"
#include "dnn_node/dnn_node.h"
#include "dnn_node/util/image_proc.h"
#include "hbm_img_msgs/msg/hbm_msg1080_p.hpp"
#include "hobot_cv/hobotcv_imgproc.h"
#include "sensor_msgs/msg/image.hpp"

#include "rapidjson/document.h"
#include "rapidjson/istreamwrapper.h"
#include "rapidjson/writer.h"

#include "racing_obstacle_detection/parser.h"
#include "racing_obstacle_detection/image_utils.h"
#include "rclcpp/qos.hpp"

// 使用hobotcv resize nv12格式图片，固定图片宽高比


int ResizeNV12Img(const char* in_img_data,
                  const int& in_img_height,
                  const int& in_img_width,
                  const int& scaled_img_height,
                  const int& scaled_img_width,
                  cv::Mat& out_img,
                  float& ratio) {
  cv::Mat src(
      in_img_height * 3 / 2, in_img_width, CV_8UC1, (void*)(in_img_data));
  float ratio_w =
      static_cast<float>(in_img_width) / static_cast<float>(scaled_img_width);
  float ratio_h =
      static_cast<float>(in_img_height) / static_cast<float>(scaled_img_height);
  float dst_ratio = std::max(ratio_w, ratio_h);
  int resized_width, resized_height;
  if (dst_ratio == ratio_w) {
    resized_width = scaled_img_width;
    resized_height = static_cast<float>(in_img_height) / dst_ratio;
  } else if (dst_ratio == ratio_h) {
    resized_width = static_cast<float>(in_img_width) / dst_ratio;
    resized_height = scaled_img_height;
  }

  // hobot_cv要求输出宽度为16的倍数
  int remain = resized_width % 16;
  if (remain != 0) {
    //向下取16倍数，重新计算缩放系数
    resized_width -= remain;
    dst_ratio = static_cast<float>(in_img_width) / resized_width;
    resized_height = static_cast<float>(in_img_height) / dst_ratio;
  }
  //高度向下取偶数
  resized_height =
      resized_height % 2 == 0 ? resized_height : resized_height - 1;
  ratio = dst_ratio;

  return hobot_cv::hobotcv_resize(
      src, in_img_height, in_img_width, out_img, resized_height, resized_width);
}

int InitClassNames(const std::string &cls_name_file,hobot::dnn_node::racing_obstacle_detection::PTQYolo5Config &yolo5_config) {
  std::ifstream fi(cls_name_file);
  if (fi) {
    yolo5_config.class_names.clear();
    std::string line;
    while (std::getline(fi, line)) {
      yolo5_config.class_names.push_back(line);
    }
    int size = yolo5_config.class_names.size();
    if(size != yolo5_config.class_num){
      RCLCPP_ERROR(rclcpp::get_logger("Yolo5_detection_parser"),
                 "class_names length %d is not equal to class_num %d",
                 size, yolo5_config.class_num);
      return -1;
    }
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("Yolo5_detection_parser"),
                 "can not open cls name file: %s",
                 cls_name_file.c_str());
    return -1;
  }
  return 0;
}

int InitClassNum(const int &class_num,hobot::dnn_node::racing_obstacle_detection::PTQYolo5Config &yolo5_config) {
  if(class_num > 0){
    yolo5_config.class_num = class_num;
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("Yolo5_detection_parser"),
                 "class_num = %d is not allowed, only support class_num > 0",
                 class_num);
    return -1;
  }
  return 0;
}

void LoadConfig(const std::string &config_file,hobot::dnn_node::racing_obstacle_detection::PTQYolo5Config &yolo5_config) {
  if (config_file.empty()) {
    RCLCPP_ERROR(rclcpp::get_logger("LoadConfig"),
                 "Config file [%s] is empty!",
                 config_file.data());
    return;
  }
  // Parsing config
  std::ifstream ifs(config_file.c_str());
  if (!ifs) {
    RCLCPP_ERROR(rclcpp::get_logger("LoadConfig"),
                 "Read config file [%s] fail!",
                 config_file.data());
    return;
  }
  rapidjson::IStreamWrapper isw(ifs);
  rapidjson::Document document;
  document.ParseStream(isw);
  if (document.HasParseError()) {
    RCLCPP_ERROR(rclcpp::get_logger("LoadConfig"),
                 "Parsing config file %s failed",
                 config_file.data());
    return;
  }

  if (document.HasMember("class_num")){
    int class_num = document["class_num"].GetInt();
    if (InitClassNum(class_num,yolo5_config) < 0) {
      return;
    }
  }
  if (document.HasMember("cls_names_list")) {
    std::string cls_name_file = document["cls_names_list"].GetString();
    if (InitClassNames(cls_name_file,yolo5_config) < 0) {
      return;
    }
  }
  return;
}

struct ObstacleDetectionNodeOutput : public hobot::dnn_node::DnnNodeOutput {
  // 缩放比例系数，原图和模型输入分辨率的比例。
  float ratio = 0.95238;
};

// 继承DnnNode虚基类，创建算法推理节点
class ObstacleDetectionNode : public hobot::dnn_node::DnnNode {
 public:
  ObstacleDetectionNode(const std::string& node_name = "ObstacleDetectionNode",
                const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

 protected:
  // 实现基类的纯虚接口，用于配置Node参数
  int SetNodePara() override;
  // 实现基类的虚接口，将解析后结构化的算法输出数据封装成ROS Msg后发布
  int PostProcess(const std::shared_ptr<hobot::dnn_node::DnnNodeOutput>&
                      node_output) override;

 private:
  // 算法输入图片数据的宽和高
  int model_input_width_ = -1;
  int model_input_height_ = -1;

  // 图片消息订阅者
  rclcpp::Subscription<hbm_img_msgs::msg::HbmMsg1080P>::ConstSharedPtr
      hbm_img_subscription_ = nullptr;
  rclcpp::Subscription<sensor_msgs::msg::Image>::ConstSharedPtr
      ros_img_subscription_ = nullptr;
  // 算法推理结果消息发布者
  rclcpp::Publisher<ai_msgs::msg::PerceptionTargets>::SharedPtr msg_publisher_ =
      nullptr;

  std::string sub_img_topic_ = "/hbmem_img";
  std::string config_file_ = "config/yolov5sconfig_simulation.json";
  bool is_shared_mem_sub_ = true;

  // {{{10, 13}, {16, 30}, {33, 23}},
  // {{30, 61}, {62, 45}, {59, 119}},
  // {{116, 90}, {156, 198}, {373, 326}}},

    //   {{{8.7,150.25}, {38.5,57.9}, {57.5,78.0}},
    //  {{76.5,105.9}, {102.1,146.3}, {141.3 ,194.3}},
    //  {{77.6, 406.5}, {209.7,283.7}, {314.7,446.7}}},
  hobot::dnn_node::racing_obstacle_detection::PTQYolo5Config yolo5_config_ = {
    {8, 16, 32},
    {{{37.21875,51.75}, {26.5,174.5}, {58.15625,81.0625}},
     {{81.4375,112.375}, {116.1875, 154.75}, {170.25, 122.0625}},
     {{106.0625, 266.0}, {185.0, 226.5}, {255.875, 365.0}}},
    2,
    {"line","zt"}};

  // 图片消息订阅回调
  void FeedHbmImg(const hbm_img_msgs::msg::HbmMsg1080P::ConstSharedPtr msg);
  void FeedImg(const sensor_msgs::msg::Image::ConstSharedPtr msg);
};

ObstacleDetectionNode::ObstacleDetectionNode(const std::string& node_name,
                             const rclcpp::NodeOptions& options)
    : hobot::dnn_node::DnnNode(node_name, options) {
  
  this->declare_parameter<std::string>("sub_img_topic", sub_img_topic_);
  this->declare_parameter<std::string>("config_file", config_file_);
  this->declare_parameter<bool>("is_shared_mem_sub", is_shared_mem_sub_);

  this->get_parameter<std::string>("sub_img_topic", sub_img_topic_);
  this->get_parameter<std::string>("config_file", config_file_);
  this->get_parameter<bool>("is_shared_mem_sub", is_shared_mem_sub_);

  // Init中使用ObstacleDetectionNode子类实现的SetNodePara()方法进行算法推理的初始化
  if (Init() != 0 ||
      GetModelInputSize(0, model_input_width_, model_input_height_) < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("ObstacleDetectionNode"), "Node init fail!");
    rclcpp::shutdown();
  }
  LoadConfig(config_file_,yolo5_config_);

  rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();

  // 创建消息订阅者，从摄像头节点订阅图像消息
  if (is_shared_mem_sub_ == true){
    hbm_img_subscription_ =
        this->create_subscription<hbm_img_msgs::msg::HbmMsg1080P>(
            sub_img_topic_,
            qos_profile,
            std::bind(&ObstacleDetectionNode::FeedHbmImg, this, std::placeholders::_1));
  } else {
    ros_img_subscription_ =
        this->create_subscription<sensor_msgs::msg::Image>(
            sub_img_topic_,
            1,
            std::bind(&ObstacleDetectionNode::FeedImg, this, std::placeholders::_1));
  }
  // 创建消息发布者，发布算法推理消息
  msg_publisher_ = this->create_publisher<ai_msgs::msg::PerceptionTargets>(
      "/origincar_competition", 10);
}

int ObstacleDetectionNode::SetNodePara() {
  if (!dnn_node_para_ptr_) return -1;
  // 指定算法推理使用的模型文件路径
  std::ifstream ifs(config_file_.c_str());
  if (!ifs) {
    RCLCPP_ERROR(rclcpp::get_logger("SetNodePara"),
                 "Read config file [%s] fail!",
                 config_file_.data());
    return -1;
  }
  rapidjson::IStreamWrapper isw(ifs);
  rapidjson::Document document;
  document.ParseStream(isw);
  if (document.HasParseError()) {
    RCLCPP_ERROR(rclcpp::get_logger("SetNodePara"),
                 "Parsing config file %s failed",
                 config_file_.data());
    return -1;
  }

  std::string model_file;
  if (document.HasMember("model_file")) {
    model_file = document["model_file"].GetString();
  }
  dnn_node_para_ptr_->model_file = model_file;
  // 指定算法推理任务类型
  // 本示例使用的人体检测算法输入为单张图片，对应的算法推理任务类型为ModelInferType
  // 只有当算法输入为图片和roi（Region of
  // Interest，例如目标的检测框）时，算法推理任务类型为ModelRoiInferType
  dnn_node_para_ptr_->model_task_type =
      hobot::dnn_node::ModelTaskType::ModelInferType;
  // 指定算法推理使用的任务数量，YOLOv5算法推理耗时较长，指定使用4个任务进行推理
  dnn_node_para_ptr_->task_num = 1;
  // 通过bpu_core_ids参数指定算法推理使用的BPU核，使用负载均衡模式
  dnn_node_para_ptr_->bpu_core_ids.push_back(HB_BPU_CORE_0);
  return 0;
}

void ObstacleDetectionNode::FeedHbmImg(
    const hbm_img_msgs::msg::HbmMsg1080P::ConstSharedPtr img_msg) {
  if (!rclcpp::ok() || !img_msg) {
    return;
  }
  // RCLCPP_ERROR(rclcpp::get_logger("ObstacleDetectionNode"), "get img_msg");

  // 1 对订阅到的图片消息进行验证，本示例只支持处理NV12格式图片数据
  // 如果是其他格式图片，订阅hobot_codec解码/转码后的图片消息
  if ("nv12" !=
      std::string(reinterpret_cast<const char*>(img_msg->encoding.data()))) {
    RCLCPP_ERROR(rclcpp::get_logger("ObstacleDetectionNode"),
                 "Only support nv12 img encoding! Using hobot codec to process "
                 "%d encoding img.",
                 img_msg->encoding.data());
    return;
  }

  // 2 创建算法输出数据，填充消息头信息，用于推理完成后AI结果的发布
  auto dnn_output = std::make_shared<ObstacleDetectionNodeOutput>();
  dnn_output->msg_header = std::make_shared<std_msgs::msg::Header>();
  dnn_output->msg_header->set__frame_id(std::to_string(img_msg->index));
  dnn_output->msg_header->set__stamp(img_msg->time_stamp);

  // 3 算法前处理，即创建算法输入数据
  std::shared_ptr<hobot::dnn_node::NV12PyramidInput> pyramid = nullptr;
  if (img_msg->height != static_cast<uint32_t>(model_input_height_) ||
      img_msg->width != static_cast<uint32_t>(model_input_width_)) {
    // 3.1 订阅到的图片和算法输入分辨率不一致，需要做resize处理
    cv::Mat out_img;
    if (ResizeNV12Img(reinterpret_cast<const char*>(img_msg->data.data()),
                      img_msg->height,
                      img_msg->width,
                      model_input_height_,
                      model_input_width_,
                      out_img,
                      dnn_output->ratio) < 0) {
      RCLCPP_ERROR(rclcpp::get_logger("ObstacleDetectionNode"),
                   "Resize nv12 img fail!");
      return;
    }

    uint32_t out_img_width = out_img.cols;
    uint32_t out_img_height = out_img.rows * 2 / 3;
    // 3.2 根据算法输入图片分辨率，使用hobot_dnn中提供的方法创建算法输入数据
    pyramid = hobot::dnn_node::ImageProc::GetNV12PyramidFromNV12Img(
        reinterpret_cast<const char*>(out_img.data),
        out_img_height,
        out_img_width,
        model_input_height_,
        model_input_width_);
  } else {
    // 3.3
    // 不需要进行resize，直接根据算法输入图片分辨率，使用hobot_dnn中提供的方法创建算法输入数据
    pyramid = hobot::dnn_node::ImageProc::GetNV12PyramidFromNV12Img(
        reinterpret_cast<const char*>(img_msg->data.data()),
        img_msg->height,
        img_msg->width,
        model_input_height_,
        model_input_width_);
  }
  // 3.4 校验算法输入数据
  if (!pyramid) {
    RCLCPP_ERROR(rclcpp::get_logger("ObstacleDetectionNode"), "Get pym fail");
    return;
  }
  // 3.5 将算法输入数据转成dnn node推理输入的格式
  auto inputs =
      std::vector<std::shared_ptr<hobot::dnn_node::DNNInput>>{pyramid};

  // 4
  // 使用创建的算法输入和输出数据，以异步模式运行推理，推理结果通过PostProcess接口回调返回
  if (Run(inputs, dnn_output, nullptr, true) < 0) {
    RCLCPP_INFO(rclcpp::get_logger("ObstacleDetectionNode"), "Run predict fail!");
  }
}

void ObstacleDetectionNode::FeedImg(
    const sensor_msgs::msg::Image::ConstSharedPtr img_msg) {
  if (!rclcpp::ok() || !img_msg) {
    return;
  }
  // 2 创建算法输出数据，填充消息头信息，用于推理完成后AI结果的发布
  auto dnn_output = std::make_shared<ObstacleDetectionNodeOutput>();
  dnn_output->msg_header = std::make_shared<std_msgs::msg::Header>();
  dnn_output->msg_header->set__frame_id(img_msg->header.frame_id);
  dnn_output->msg_header->set__stamp(img_msg->header.stamp);
  auto cv_img = cv_bridge::cvtColorForDisplay(cv_bridge::toCvShare(img_msg), "bgr8");
  // 3 算法前处理，即创建算法输入数据
  std::shared_ptr<hobot::dnn_node::NV12PyramidInput> pyramid = nullptr;
  pyramid = ImageUtils::GetNV12Pyramid(
        cv_img->image, model_input_height_, model_input_width_);
  // 3.4 校验算法输入数据
  if (!pyramid) {
    RCLCPP_ERROR(rclcpp::get_logger("ObstacleDetectionNode"), "Get pym fail");
    return;
  }
  // // 3.5 将算法输入数据转成dnn node推理输入的格式
  auto inputs =
      std::vector<std::shared_ptr<hobot::dnn_node::DNNInput>>{pyramid};

  // // 4
  // // 使用创建的算法输入和输出数据，以异步模式运行推理，推理结果通过PostProcess接口回调返回
  if (Run(inputs, dnn_output, nullptr, false) < 0) {
    RCLCPP_INFO(rclcpp::get_logger("ObstacleDetectionNode"), "Run predict fail!");
  }
}


// 推理结果回调，解析算法输出，通过ROS Msg发布消息
int ObstacleDetectionNode::PostProcess(
    const std::shared_ptr<hobot::dnn_node::DnnNodeOutput>& node_output) {
  if (!rclcpp::ok()) {
    return 0;
  }

  // 后处理开始时间
  auto tp_start = std::chrono::system_clock::now();

  // 1 创建用于发布推理结果的ROS Msg
  ai_msgs::msg::PerceptionTargets::UniquePtr pub_data(
      new ai_msgs::msg::PerceptionTargets());

  // 2 将推理输出对应图片的消息头填充到ROS Msg
  pub_data->set__header(*node_output->msg_header);

  // 3 使用自定义的Parse解析方法，解析算法输出的DNNTensor类型数据
  // 3.1
  // 创建解析输出数据，输出YoloV5Result是自定义的算法输出数据类型，results的维度等于检测出来的目标数
  std::vector<std::shared_ptr<hobot::dnn_node::racing_obstacle_detection::YoloV5Result>>
      results;

  // 3.2 开始解析
  if (hobot::dnn_node::racing_obstacle_detection::Parse(node_output, results, yolo5_config_) < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("ObstacleDetectionNode"),
                 "Parse node_output fail!");
    return -1;
  }

  // 3.3 使用解析后的数据填充到ROS Msg
  for (auto& rect : results) {
    if (!rect) continue;
    if (rect->xmin < 0) rect->xmin = 0;
    if (rect->ymin < 0) rect->ymin = 0;
    if (rect->xmax >= model_input_width_) {
      rect->xmax = model_input_width_ - 1;
    }
    if (rect->ymax >= model_input_height_) {
      rect->ymax = model_input_height_ - 1;
    }

    std::stringstream ss;
    ss << "det rect: " << rect->xmin << " " << rect->ymin << " " << rect->xmax
       << " " << rect->ymax << ", det type: " << rect->class_name
       << ", score:" << rect->score;
    RCLCPP_INFO(rclcpp::get_logger("ObstacleDetectionNode"), "%s", ss.str().c_str());

    ai_msgs::msg::Roi roi;
    roi.rect.set__x_offset(rect->xmin);
    roi.rect.set__y_offset(rect->ymin);
    roi.rect.set__width(rect->xmax - rect->xmin);
    roi.rect.set__height(rect->ymax - rect->ymin);
    roi.set__confidence(rect->score);

    ai_msgs::msg::Target target;
    target.set__type(rect->class_name);
    target.rois.emplace_back(roi);
    pub_data->targets.emplace_back(std::move(target));
  }

  // 4 坐标映射
  auto sample_node_output =
      std::dynamic_pointer_cast<ObstacleDetectionNodeOutput>(node_output);
  if (!sample_node_output) {
    RCLCPP_ERROR(rclcpp::get_logger("ObstacleDetectionNode"),
                 "Cast dnn node output fail!");
    return -1;
  }
  if (sample_node_output->ratio != 1.0) {
    // 前处理有对图片进行resize，需要将坐标映射到对应的订阅图片分辨率
    for (auto& target : pub_data->targets) {
      for (auto& roi : target.rois) {
        roi.rect.x_offset *= sample_node_output->ratio;
        roi.rect.y_offset *= sample_node_output->ratio;
        roi.rect.width *= sample_node_output->ratio;
        roi.rect.height *= sample_node_output->ratio;
      }
    }
  }

  // 5 将算法推理输出帧率填充到ROS Msg
  if (node_output->rt_stat) {
    pub_data->set__fps(round(node_output->rt_stat->output_fps));
    // 如果算法推理统计有更新，输出算法输入和输出的帧率统计、推理耗时
    if (node_output->rt_stat->fps_updated) {
      // 后处理结束时间
      auto tp_now = std::chrono::system_clock::now();
      auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(
                          tp_now - tp_start)
                          .count();
      RCLCPP_WARN(rclcpp::get_logger("ObstacleDetectionNode"),
                  "input fps: %.2f, out fps: %.2f, infer time ms: %d, "
                  "post process time ms: %d",
                  node_output->rt_stat->input_fps,
                  node_output->rt_stat->output_fps,
                  node_output->rt_stat->infer_time_ms,
                  interval);
    }
  }

  // 6 发布ROS Msg
  msg_publisher_->publish(std::move(pub_data));

  return 0;
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstacleDetectionNode>());
  rclcpp::shutdown();
  return 0;
}
