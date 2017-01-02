/**\file assembly_manager.cpp
 * \brief Description...
 *
 * @version 1.0
 * @author carloscosta
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include <assembly_projection_mapping_teaching/assembly_manager.h>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

namespace assembly_projection_mapping_teaching {

// =============================================================================  <public-section>  ============================================================================
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <constructors-destructor>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
AssemblyManager::AssemblyManager() :
		current_assembly_step_(0),
		video_paused_(false),
		video_seek_start_position_(0.0),
		button_highlight_time_sec_(0.5) {}
AssemblyManager::~AssemblyManager() {}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constructors-destructor>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <member-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
bool AssemblyManager::loadConfigurationFromParameterServer(ros::NodeHandlePtr& _node_handle, ros::NodeHandlePtr& _private_node_handle, const std::string& _configuration_namespace) {
	node_handle_ = _node_handle;
	private_node_handle_ = _private_node_handle;

	private_node_handle_->param("media_folder_path", media_folder_path_, std::string(""));
	if (media_folder_path_.empty()) {
		ROS_FATAL("Missing media folder path!");
		return false;
	}

	private_node_handle_->param("videos_start_paused", video_paused_, false);
	private_node_handle_->param("video_seek_start_position", video_seek_start_position_, 0.0);
	private_node_handle_->param("video_seek_end_position", video_seek_end_position_, 1.0);
	private_node_handle_->param("button_highlight_time_sec", button_highlight_time_sec_, 0.5);

	XmlRpc::XmlRpcValue steps;
	if (_private_node_handle->getParam(_configuration_namespace, steps) && steps.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
		for (XmlRpc::XmlRpcValue::iterator it = steps.begin(); it != steps.end(); ++it) {
			std::string step_name = it->first;
			if (step_name.find("step") != std::string::npos) {
				std::string text_path;
				std::string video_path;
				private_node_handle_->param(_configuration_namespace + step_name + "/text", text_path, std::string(""));
				private_node_handle_->param(_configuration_namespace + step_name + "/video", video_path, std::string(""));
				if (!text_path.empty() && !video_path.empty()) {
					assembly_text_images_paths_.push_back(media_folder_path_ + "/" + text_path);
					assembly_video_paths_.push_back(media_folder_path_ + "/" + video_path);
				}
			}
		}
	}

	return !assembly_text_images_paths_.empty() && !assembly_video_paths_.empty();
}

void AssemblyManager::start() {
	setupPublishers();
	publishStepCounter(assembly_text_images_paths_.size(), publisher_set_third_number_path_, publisher_set_fourth_number_path_);
	publishCurrentAssemblyStepContent("first", publisher_set_first_button_path_, false);
	startSubscribers();
	ros::spin();
}

void AssemblyManager::setupPublishers() {
	publisher_set_text_path_			= node_handle_->advertise<std_msgs::String>("/gazebo_projecion_mapping/text/set_image_path", 1, true);
	publisher_set_video_path_ 			= node_handle_->advertise<std_msgs::String>("/gazebo_projecion_mapping/video/set_video_path", 1, true);
	publisher_set_video_seek_ 			= node_handle_->advertise<std_msgs::Float64>("/gazebo_projecion_mapping/video/set_video_seek", 1, true);
	publisher_set_video_paused_ 		= node_handle_->advertise<std_msgs::Bool>("/gazebo_projecion_mapping/video/set_video_paused", 1, true);
	publisher_set_first_button_path_ 	= node_handle_->advertise<std_msgs::String>("/gazebo_projecion_mapping/first_button/set_image_path", 1, true);
	publisher_set_previous_button_path_ = node_handle_->advertise<std_msgs::String>("/gazebo_projecion_mapping/previous_button/set_image_path", 1, true);
	publisher_set_next_button_path_ 	= node_handle_->advertise<std_msgs::String>("/gazebo_projecion_mapping/next_button/set_image_path", 1, true);
	publisher_set_last_button_path_ 	= node_handle_->advertise<std_msgs::String>("/gazebo_projecion_mapping/last_button/set_image_path", 1, true);
	publisher_set_first_number_path_ 	= node_handle_->advertise<std_msgs::String>("/gazebo_projecion_mapping/first_number/set_image_path", 1, true);
	publisher_set_second_number_path_ 	= node_handle_->advertise<std_msgs::String>("/gazebo_projecion_mapping/second_number/set_image_path", 1, true);
	publisher_set_third_number_path_ 	= node_handle_->advertise<std_msgs::String>("/gazebo_projecion_mapping/third_number/set_image_path", 1, true);
	publisher_set_fourth_number_path_ 	= node_handle_->advertise<std_msgs::String>("/gazebo_projecion_mapping/fourth_number/set_image_path", 1, true);
}

void AssemblyManager::startSubscribers() {
	subscriber_occupancy_detection_video_paused_ 	= node_handle_->subscribe("/occupancy_detection/occupancy_detection_video_paused", 1, &AssemblyManager::processVideoPausedMsg, this);
	subscriber_occupancy_detection_video_seek_ 		= node_handle_->subscribe("/occupancy_detection/occupancy_detection_video_seek", 1, &AssemblyManager::processVideoSeekMsg, this);
	subscriber_occupancy_detection_first_button_ 	= node_handle_->subscribe("/occupancy_detection/occupancy_detection_first_button", 1, &AssemblyManager::processFirstButtonMsg, this);
	subscriber_occupancy_detection_previous_button_ = node_handle_->subscribe("/occupancy_detection/occupancy_detection_previous_button", 1, &AssemblyManager::processPreviousButtonMsg, this);
	subscriber_occupancy_detection_next_button_ 	= node_handle_->subscribe("/occupancy_detection/occupancy_detection_next_button", 1, &AssemblyManager::processNextButtonMsg, this);
	subscriber_occupancy_detection_last_button_ 	= node_handle_->subscribe("/occupancy_detection/occupancy_detection_last_button", 1, &AssemblyManager::processLastButtonMsg, this);
}

void AssemblyManager::processVideoPausedMsg(const geometry_msgs::PointStampedConstPtr& _msg) {
	video_paused_ = !video_paused_;
	std_msgs::Bool msg_to_publish;
	msg_to_publish.data = video_paused_;
	publisher_set_video_paused_.publish(msg_to_publish);
}

void AssemblyManager::processVideoSeekMsg(const geometry_msgs::PointStampedConstPtr& _msg) {
	double video_seek_length = video_seek_end_position_ - video_seek_start_position_;
	double video_position_seek = _msg->point.x - video_seek_start_position_;
	if (video_seek_length > 0 && video_position_seek >= 0 && video_position_seek <= video_seek_length) {
		std_msgs::Float64 msg_to_publish;
		msg_to_publish.data = video_position_seek / video_seek_length;
		publisher_set_video_seek_.publish(msg_to_publish);
	}
}

void AssemblyManager::processFirstButtonMsg(const geometry_msgs::PointStampedConstPtr& _msg) {
	if (assembly_text_images_paths_.size() > 0 && current_assembly_step_ > 0) {
		current_assembly_step_ = 0;
		publishCurrentAssemblyStepContent("first", publisher_set_first_button_path_);
	}
}

void AssemblyManager::processPreviousButtonMsg(const geometry_msgs::PointStampedConstPtr& _msg) {
	if (assembly_text_images_paths_.size() > 0 && current_assembly_step_ > 0) {
		--current_assembly_step_;
		publishCurrentAssemblyStepContent("previous", publisher_set_previous_button_path_);
	}
}

void AssemblyManager::processNextButtonMsg(const geometry_msgs::PointStampedConstPtr& _msg) {
	if (assembly_text_images_paths_.size() > 0 && current_assembly_step_ < assembly_text_images_paths_.size() - 1) {
		++current_assembly_step_;
		publishCurrentAssemblyStepContent("next", publisher_set_next_button_path_);
	}
}

void AssemblyManager::processLastButtonMsg(const geometry_msgs::PointStampedConstPtr& _msg) {
	if (assembly_text_images_paths_.size() > 0 && current_assembly_step_ < assembly_text_images_paths_.size() - 1) {
		current_assembly_step_ = assembly_text_images_paths_.size() - 1;
		publishCurrentAssemblyStepContent("last", publisher_set_last_button_path_);
	}
}

void AssemblyManager::publishCurrentAssemblyStepContent(const std::string& _button_name, ros::Publisher& _image_path_publisher, bool publish_highlighted_button) {
	if (publish_highlighted_button) {
		std_msgs::String button_image_path_selected;
		button_image_path_selected.data = media_folder_path_ + "/images/buttons/" + _button_name + "_selected.png";
		_image_path_publisher.publish(button_image_path_selected);
	}

	publishStepCounter(current_assembly_step_ + 1, publisher_set_first_number_path_, publisher_set_second_number_path_);

	std_msgs::String text_image_path;
	text_image_path.data = assembly_text_images_paths_[current_assembly_step_];
	publisher_set_text_path_.publish(text_image_path);

	std_msgs::String video_path;
	video_path.data = assembly_video_paths_[current_assembly_step_];
	publisher_set_video_path_.publish(video_path);

	std_msgs::Bool msg_to_publish;
	msg_to_publish.data = false;
	publisher_set_video_paused_.publish(msg_to_publish);
	video_paused_ = false;

	ros::Duration button_highlight_time_ms(button_highlight_time_sec_);
	button_highlight_time_ms.sleep();

	std_msgs::String button_image_path;
	button_image_path.data = media_folder_path_ + "/images/buttons/" + _button_name + ".png";
	_image_path_publisher.publish(button_image_path);
}

void AssemblyManager::publishStepCounter(size_t _number, ros::Publisher& _first_number_publisher, ros::Publisher& _second_number_publisher) {
	char first_number = '0';
	char second_number = '0';

	if (_number > 99) {
		first_number = 9;
		second_number = 9;
	} else if (_number > 0) {
		std::stringstream number_ss;
		number_ss << _number;
		std::string number_str = number_ss.str();
		if (number_str.size() == 1) {
			second_number = number_str[0];
		}
		if (number_str.size() == 2) {
			first_number = number_str[0];
			second_number = number_str[1];
		}
	}

	std_msgs::String first_number_path;
	std::stringstream first_number_path_ss;
	first_number_path_ss << media_folder_path_ << "/images/numbers/" << first_number << ".png";
	first_number_path.data = first_number_path_ss.str();
	_first_number_publisher.publish(first_number_path);

	std_msgs::String second_number_path;
	std::stringstream second_number_path_ss;
	second_number_path_ss << media_folder_path_ << "/images/numbers/" << second_number << ".png";
	second_number_path.data = second_number_path_ss.str();
	_second_number_publisher.publish(second_number_path);
}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </member-functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// =============================================================================  </public-section>  ===========================================================================

// =============================================================================   <protected-section>   =======================================================================
// =============================================================================   </protected-section>  =======================================================================

} /* namespace assembly_projection_mapping_teaching */

