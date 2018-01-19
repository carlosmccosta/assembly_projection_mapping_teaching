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
		video_seek_end_position_(1.0),
		button_highlight_time_sec_(0.5),
		z_offset_for_hiding_gazebo_models_(10.0) {}
AssemblyManager::~AssemblyManager() {}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constructors-destructor>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <member-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
bool AssemblyManager::loadConfigurationFromParameterServer(ros::NodeHandlePtr& _node_handle, ros::NodeHandlePtr& _private_node_handle, const std::string& _configuration_namespace) {
	node_handle_ = _node_handle;
	private_node_handle_ = _private_node_handle;

	private_node_handle_->param("media_folder_path", media_folder_path_, std::string(""));
	private_node_handle_->param("media_folder_path_for_user_interface", media_folder_path_for_user_interface_, std::string(""));
	if (media_folder_path_.empty()) {
		ROS_FATAL("Missing media folder path!");
		return false;
	}

	if (media_folder_path_for_user_interface_.empty())
		media_folder_path_for_user_interface_ = media_folder_path_;

	private_node_handle_->param("videos_start_paused", video_paused_, false);
	private_node_handle_->param("video_seek_point_axis", video_seek_point_axis_, std::string("x"));
	private_node_handle_->param("video_seek_start_position", video_seek_start_position_, 0.0);
	private_node_handle_->param("video_seek_end_position", video_seek_end_position_, 1.0);
	private_node_handle_->param("button_highlight_time_sec", button_highlight_time_sec_, 0.5);
	private_node_handle_->param("z_offset_for_hiding_gazebo_models", z_offset_for_hiding_gazebo_models_, 10.0);

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

					std::string models_to_show;
					private_node_handle_->param(_configuration_namespace + step_name + "/gazebo_models_to_show", models_to_show, std::string(""));
					std::vector<std::string> models_to_show_tokens;
					splitString(models_to_show, '+', models_to_show_tokens);
					gazebo_models_to_show_in_single_step_.push_back(models_to_show_tokens);

					std::string models_to_hide;
					private_node_handle_->param(_configuration_namespace + step_name + "/gazebo_models_to_hide", models_to_hide, std::string(""));
					std::vector<std::string> models_to_hide_tokens;
					splitString(models_to_hide, '+', models_to_hide_tokens);
					gazebo_models_to_hide_in_single_step_.push_back(models_to_hide_tokens);

					std::string frame_id;
					private_node_handle_->param(_configuration_namespace + step_name + "/tf_offset_to_publish_frame_id", frame_id, std::string(""));
					tf_offset_to_publish_frame_id_in_single_step_.push_back(frame_id);

					std::string child_frame_id;
					private_node_handle_->param(_configuration_namespace + step_name + "/tf_offset_to_publish_child_frame_id", child_frame_id, std::string(""));
					tf_offset_to_publish_child_frame_id_in_single_step_.push_back(child_frame_id);
				}
			}
		}
	}

	return !assembly_text_images_paths_.empty() && !assembly_video_paths_.empty();
}

void AssemblyManager::start() {
	setupPublishers();
	setupGazeboCommunication();
	publishStepCounter(assembly_text_images_paths_.size(), publisher_set_third_number_path_, publisher_set_fourth_number_path_);
	publishCurrentAssemblyStepContent("first", publisher_set_first_button_path_, false);
	updateGazeboModels(ros::Time::now(), gazebo_models_to_show_in_single_step_[0], -z_offset_for_hiding_gazebo_models_);
	updateGazeboModels(ros::Time::now(), gazebo_models_to_hide_in_single_step_[0], z_offset_for_hiding_gazebo_models_);
	publishStepTF(ros::Time::now(), tf_offset_to_publish_frame_id_in_single_step_[0], tf_offset_to_publish_child_frame_id_in_single_step_[0], -z_offset_for_hiding_gazebo_models_);
	startSubscribers();
	ros::spin();
}

void AssemblyManager::setupGazeboCommunication() {
	std::string str;
	private_node_handle_->param("gazebo_get_model_state_service", str, std::string("/gazebo/get_model_state"));
	model_state_client_ = node_handle_->serviceClient<gazebo_msgs::GetModelState>(str);

	private_node_handle_->param("gazebo_set_model_state_topic", str, std::string("/gazebo/set_model_state"));
	model_state_publisher_ = node_handle_->advertise<gazebo_msgs::ModelState>(str, 1, true);
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
	double video_position_seek;
	if (video_seek_point_axis_ == "x") {
		video_position_seek = _msg->point.x - video_seek_start_position_;
	} else if (video_seek_point_axis_ == "y") {
		video_position_seek = _msg->point.y - video_seek_start_position_;
	} else {
		video_position_seek = _msg->point.z - video_seek_start_position_;
	}

	if (video_seek_length > 0 && video_position_seek >= 0 && video_position_seek <= video_seek_length) {
		std_msgs::Float64 msg_to_publish;
		msg_to_publish.data = video_position_seek / video_seek_length;
		publisher_set_video_seek_.publish(msg_to_publish);
	}
}

void AssemblyManager::processFirstButtonMsg(const geometry_msgs::PointStampedConstPtr& _msg) {
	if (assembly_text_images_paths_.size() > 0 && current_assembly_step_ > 0) {
		for (int i = current_assembly_step_; i > 1; --i) {
			processPreviousButton(false);
		}

		processPreviousButton(true);
	}
}

void AssemblyManager::processPreviousButtonMsg(const geometry_msgs::PointStampedConstPtr& _msg) {
	processPreviousButton(true);
}

void AssemblyManager::processPreviousButton(bool _publish_step_content) {
	if (assembly_text_images_paths_.size() > 0 && current_assembly_step_ > 0) {
		updateGazeboModels(ros::Time::now(), gazebo_models_to_show_in_single_step_[current_assembly_step_], z_offset_for_hiding_gazebo_models_);
		updateGazeboModels(ros::Time::now(), gazebo_models_to_hide_in_single_step_[current_assembly_step_], -z_offset_for_hiding_gazebo_models_);

		publishStepTF(ros::Time::now(), tf_offset_to_publish_frame_id_in_single_step_[current_assembly_step_], tf_offset_to_publish_child_frame_id_in_single_step_[current_assembly_step_], 0);
		--current_assembly_step_;
		publishStepTF(ros::Time::now(), tf_offset_to_publish_frame_id_in_single_step_[current_assembly_step_], tf_offset_to_publish_child_frame_id_in_single_step_[current_assembly_step_], -z_offset_for_hiding_gazebo_models_);

		if (_publish_step_content)
			publishCurrentAssemblyStepContent("previous", publisher_set_previous_button_path_);
	}
}

void AssemblyManager::processNextButtonMsg(const geometry_msgs::PointStampedConstPtr& _msg) {
	processNextButton(true);
}

void AssemblyManager::processNextButton(bool _publish_step_content) {
	if (assembly_text_images_paths_.size() > 0 && current_assembly_step_ < assembly_text_images_paths_.size() - 1) {
		publishStepTF(ros::Time::now(), tf_offset_to_publish_frame_id_in_single_step_[current_assembly_step_], tf_offset_to_publish_child_frame_id_in_single_step_[current_assembly_step_], 0);
		++current_assembly_step_;
		publishStepTF(ros::Time::now(), tf_offset_to_publish_frame_id_in_single_step_[current_assembly_step_], tf_offset_to_publish_child_frame_id_in_single_step_[current_assembly_step_], -z_offset_for_hiding_gazebo_models_);

		updateGazeboModels(ros::Time::now(), gazebo_models_to_show_in_single_step_[current_assembly_step_], -z_offset_for_hiding_gazebo_models_);
		updateGazeboModels(ros::Time::now(), gazebo_models_to_hide_in_single_step_[current_assembly_step_], z_offset_for_hiding_gazebo_models_);

		if (_publish_step_content)
			publishCurrentAssemblyStepContent("next", publisher_set_next_button_path_);
	}
}

void AssemblyManager::processLastButtonMsg(const geometry_msgs::PointStampedConstPtr& _msg) {
	while (current_assembly_step_ < assembly_text_images_paths_.size() - 1) {
		processNextButton(false);
	}

	processNextButton(true);
}

void AssemblyManager::publishCurrentAssemblyStepContent(const std::string& _button_name, ros::Publisher& _image_path_publisher, bool publish_highlighted_button) {
	if (publish_highlighted_button) {
		std_msgs::String button_image_path_selected;
		button_image_path_selected.data = media_folder_path_for_user_interface_ + "/images/buttons/" + _button_name + "_selected.png";
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
	button_image_path.data = media_folder_path_for_user_interface_ + "/images/buttons/" + _button_name + ".png";
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
	first_number_path_ss << media_folder_path_for_user_interface_ << "/images/numbers/" << first_number << ".png";
	first_number_path.data = first_number_path_ss.str();
	_first_number_publisher.publish(first_number_path);

	std_msgs::String second_number_path;
	std::stringstream second_number_path_ss;
	second_number_path_ss << media_folder_path_for_user_interface_ << "/images/numbers/" << second_number << ".png";
	second_number_path.data = second_number_path_ss.str();
	_second_number_publisher.publish(second_number_path);
}

void AssemblyManager::updateGazeboModels(const ros::Time& _time_stamp, const std::vector<std::string>& _model_names, double _z_offset) {
	for (size_t i = 0; i < _model_names.size(); ++i) {
		gazebo_msgs::GetModelState model_state;
		model_state.request.model_name = _model_names[i];
		if (model_state_client_.call(model_state) && model_state.response.success) {
			gazebo_msgs::ModelState new_model_state;
			new_model_state.model_name = _model_names[i];
			new_model_state.pose = model_state.response.pose;
			new_model_state.pose.position.z += _z_offset;
			new_model_state.twist = model_state.response.twist;
			model_state_publisher_.publish(new_model_state);
		}
	}
}

void AssemblyManager::publishStepTF(const ros::Time& _time_stamp, const std::string& _frame_id, const std::string& _child_frame_id, double _z_offset) {
	if (!_frame_id.empty() && !_child_frame_id.empty()) {
		geometry_msgs::TransformStamped tf;
			tf.header.stamp = _time_stamp;
			tf.header.frame_id = _frame_id;
			tf.child_frame_id = _child_frame_id;
			tf.transform.rotation.w = 1;
			tf.transform.translation.z = _z_offset;
			transform_broadcaster_.sendTransform(tf);
	}
}

void AssemblyManager::splitString(const std::string& _str, char _delimiter,  std::vector<std::string>& _tokens_out) {
	std::string str(_str);
	std::replace(str.begin(), str.end(), _delimiter, ' ');
	std::stringstream ss(str);
	std::string str_temp;
	while (ss >> str_temp && !str_temp.empty()) {
		_tokens_out.push_back(str_temp);
	}
}

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </member-functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// =============================================================================  </public-section>  ===========================================================================

// =============================================================================   <protected-section>   =======================================================================
// =============================================================================   </protected-section>  =======================================================================

} /* namespace assembly_projection_mapping_teaching */

