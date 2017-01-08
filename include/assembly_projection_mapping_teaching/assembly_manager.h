#pragma once

/**\file assembly_manager.h
 * \brief Description...
 *
 * @version 1.0
 * @author carloscosta
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>  <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include <string>
#include <sstream>
#include <vector>
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>
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

		void setupPublishers();
		void startSubscribers();

		void processVideoPausedMsg(const geometry_msgs::PointStampedConstPtr& _msg);
		void processVideoSeekMsg(const geometry_msgs::PointStampedConstPtr& _msg);
		void processFirstButtonMsg(const geometry_msgs::PointStampedConstPtr& _msg);
		void processPreviousButtonMsg(const geometry_msgs::PointStampedConstPtr& _msg);
		void processNextButtonMsg(const geometry_msgs::PointStampedConstPtr& _msg);
		void processLastButtonMsg(const geometry_msgs::PointStampedConstPtr& _msg);

		void publishCurrentAssemblyStepContent(const std::string& _button_name, ros::Publisher& _image_path_publisher, bool publish_highlighted_button = true);
		void publishStepCounter(size_t _number, ros::Publisher& _first_number_publisher, ros::Publisher& _second_number_publisher);
		void publishAssembledObjectTF(const ros::Time& _time_stamp, double _z_offset);
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
		double z_offset_for_hiding_assembled_object_;
		geometry_msgs::TransformStamped assembled_object_final_pose_;
		std::string media_folder_path_;
		std::vector<std::string> assembly_text_images_paths_;
		std::vector<std::string> assembly_video_paths_;
	// ========================================================================   </protected-section>  ========================================================================
};

} /* namespace assembly_projection_mapping_teaching */
