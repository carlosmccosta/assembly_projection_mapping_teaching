#pragma once

/**\file assembly_manager.h
 * \brief Description...
 *
 * @version 1.0
 * @author carloscosta
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>  <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include <algorithm>
#include <string>
#include <sstream>
#include <vector>
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/ModelState.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <tf2_ros/static_transform_broadcaster.h>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

namespace assembly_projection_mapping_teaching {
// ############################################################################   AssemblyManager   #############################################################################
/**
 * \brief Description...
 */
class AssemblyManager {
	// ========================================================================   <public-section>   ===========================================================================
	public:
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <typedefs>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </typedefs>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <constructors-destructor>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		AssemblyManager();
		virtual ~AssemblyManager();
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constructors-destructor>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <member-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		virtual bool loadConfigurationFromParameterServer(ros::NodeHandlePtr& _node_handle, ros::NodeHandlePtr& _private_node_handle,
				const std::string& _configuration_namespace = "");
		virtual void start();

		void setupGazeboCommunication();
		void setupPublishers();
		void startSubscribers();

		void processVideoPausedMsg(const geometry_msgs::PointStampedConstPtr& _msg);
		void processVideoSeekMsg(const geometry_msgs::PointStampedConstPtr& _msg);
		void processFirstButtonMsg(const geometry_msgs::PointStampedConstPtr& _msg);
		void processPreviousButtonMsg(const geometry_msgs::PointStampedConstPtr& _msg);
		void processPreviousButton(bool _publish_step_content = true);
		void processNextButtonMsg(const geometry_msgs::PointStampedConstPtr& _msg);
		void processNextButton(bool _publish_step_content = true);
		void processLastButtonMsg(const geometry_msgs::PointStampedConstPtr& _msg);

		void publishCurrentAssemblyStepContent(const std::string& _button_name, ros::Publisher& _image_path_publisher, bool publish_highlighted_button = true);
		void publishStepCounter(size_t _number, ros::Publisher& _first_number_publisher, ros::Publisher& _second_number_publisher);
		void updateGazeboModels(const ros::Time& _time_stamp, const std::vector<std::string>& _model_names, double _z_offset);
		void publishStepTF(const ros::Time& _time_stamp, const std::string& _frame_id, const std::string& _child_frame_id, double _z_offset);
		static void splitString(const std::string& _str, char _delimiter,  std::vector<std::string>& _tokens_out);
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </member-functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <gets>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </gets>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <sets>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </sets>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
	// ========================================================================   </public-section>  ===========================================================================

	// ========================================================================   <protected-section>   ========================================================================
	protected:
		ros::NodeHandlePtr node_handle_;
		ros::NodeHandlePtr private_node_handle_;
		ros::ServiceClient model_state_client_;
		ros::Publisher model_state_publisher_;
		tf2_ros::StaticTransformBroadcaster transform_broadcaster_;

		ros::Subscriber subscriber_occupancy_detection_video_paused_;
		ros::Subscriber subscriber_occupancy_detection_video_seek_;
		ros::Subscriber subscriber_occupancy_detection_first_button_;
		ros::Subscriber subscriber_occupancy_detection_previous_button_;
		ros::Subscriber subscriber_occupancy_detection_next_button_;
		ros::Subscriber subscriber_occupancy_detection_last_button_;

		ros::Publisher publisher_set_text_path_;
		ros::Publisher publisher_set_video_path_;
		ros::Publisher publisher_set_video_seek_;
		ros::Publisher publisher_set_video_paused_;
		ros::Publisher publisher_set_first_button_path_;
		ros::Publisher publisher_set_previous_button_path_;
		ros::Publisher publisher_set_next_button_path_;
		ros::Publisher publisher_set_last_button_path_;
		ros::Publisher publisher_set_first_number_path_;
		ros::Publisher publisher_set_second_number_path_;
		ros::Publisher publisher_set_third_number_path_;
		ros::Publisher publisher_set_fourth_number_path_;

		size_t current_assembly_step_;
		bool video_paused_;
		std::string video_seek_point_axis_;
		double video_seek_start_position_;
		double video_seek_end_position_;
		double button_highlight_time_sec_;
		double z_offset_for_hiding_gazebo_models_;
		std::string media_folder_path_;
		std::string media_folder_path_for_user_interface_;
		std::vector<std::string> assembly_text_images_paths_;
		std::vector<std::string> assembly_video_paths_;
		std::vector< std::vector<std::string> > gazebo_models_to_show_in_single_step_;
		std::vector< std::vector<std::string> > gazebo_models_to_hide_in_single_step_;
		std::vector<std::string> tf_offset_to_publish_frame_id_in_single_step_;
		std::vector<std::string> tf_offset_to_publish_child_frame_id_in_single_step_;
	// ========================================================================   </protected-section>  ========================================================================
};

} /* namespace assembly_projection_mapping_teaching */
