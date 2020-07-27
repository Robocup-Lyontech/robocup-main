/*
 * 
 *
 *  
 *      Author: jsaraydaryan
 */

#include <ros/ros.h>
#include "darknet_ros_msgs/BoundingBox.h"
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "robocup_msgs/Entity.h"
#include "robocup_msgs/EntityList.h"
#include "robocup_msgs/Entity2D.h"
#include "sensor_msgs/PointCloud2.h"
#include "convert_2d_to_3d/get3Dfrom2D.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <math.h> 
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/PointStamped.h"
#include <tf/transform_listener.h>
#include <cstdlib>
#include <string>

// Publisher used to send evidence to world model
ros::Publisher pub_3D;
ros::Publisher object_marker_pub,object_entity_pub;
ros::Subscriber sub_registered_darknetBoxes;
ros::Subscriber sub_registered_pcl;
//tf::TransformListener listener;

sensor_msgs::PointCloud2 current_pcl;

pcl::PointCloud<pcl::PointXYZ> point_pcl;
sensor_msgs::PointCloud2::ConstPtr current_cloud;

int height;
std::string FRAME_ID="";
std::string filter;
std::string target_frame,source_frame;

bool display_marker=true;


/*
* CAN BE TESTED WITH
* rosservice call /convert_2d_to_3d "{'pose':{'x':10,'y':10}}"
*/




void getPclCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud){
try{
	ROS_INFO("--> inside getPclCallback");
	//int pcl_index, rgb_h, rgb_w;
	//rgb_h = 240;
	//rgb_w = 320;
	height=cloud->height;
	//pcl_index = ( (rgb_w*cloud->height) + rgb_h);


	current_cloud=cloud;
	//std::cout << "(x,y,z) = " << point_pcl.at(pcl_index) << std::endl;
	ROS_INFO("--> GET PCL");
} catch (...) {
		ROS_WARN("Exception getPclCallback...");
    	return ;
	}
}


void addMarker(visualization_msgs::MarkerArray& m_array,float x, float y, float z, std::string label,std::string frame_id,int id) {
		visualization_msgs::Marker marker,markerTxt;
		marker.header.frame_id = frame_id;
		marker.header.stamp = ros::Time();
		marker.ns = "/world_management/pose_recorded";
		marker.id=id;

		//marker.color.a = 0.5;//initValue = 0.15
		marker.color.r = 1.0;
		marker.color.g = 0.0;
		marker.color.b = 0.0;
		marker.color.a = 0.75;
		marker.pose.position.x = x;
		marker.pose.position.y = y;
		marker.pose.orientation.w=1;
		//markerN.pose.position.z = marker.scale.z / 2.0;;
		marker.pose.position.z = z;
		marker.type = visualization_msgs::Marker::SPHERE;
		marker.scale.x=0.05;
		marker.scale.y=0.05;
		marker.scale.z=0.05;
		
		/*markerTxt=marker;
		markerTxt.id=id+1000;
		markerTxt.pose.position.y=marker.pose.position.y+0.1;
		markerTxt.text=std::to_string(id);
		markerTxt.ns = "/world_management/pose_recorder_txt";
		markerTxt.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
		*/
		m_array.markers.push_back(marker);
		//m_array.markers.push_back(markerTxt);


}

bool convert2dto3dCallback(convert_2d_to_3d::get3Dfrom2D::Request  &req,
             convert_2d_to_3d::get3Dfrom2D::Response &res){
				 if(current_cloud==0){
					 return false;
				 }
	ROS_DEBUG("Service called");
	try{
		int pcl_index,x,y,z;
		pcl::PointXYZ point;
		int rgb_w=req.pose.x;
		int rgb_h=req.pose.y;
		std::string frame_id=req.frame_id;
		//pcl::PointXYZ target = pCloud.points[CoM.x + CoM.y * pCloud.width];

		pcl_index = ( (rgb_w*current_cloud->width) + rgb_h);

		//pcl_index = ( (rgb_w*current_cloud->height) + rgb_h);
		pcl::fromROSMsg(*current_cloud,point_pcl);
		point =point_pcl.at(pcl_index);
		if(isnan(point.x) || isnan(point.y) || isnan(point.z)){
			ROS_WARN("ISNAN value x:%f,y:%f,z:%f",point.x,point.y,point.z);
			return false;
		}


		if(isinf(point.x) || isinf(point.y) || isinf(point.z)){
			ROS_WARN("ISINF value x:%f,y:%f,z:%f",point.x,point.y,point.z);
			return false;
		}

		/*geometry_msgs::PointStamped pt_stamp_in;
		pt_stamp_in.header.frame_id=""
		pt_stamp_in.point.x=point.x;
		pt_stamp_in.point.y=point.y;
		pt_stamp_in.point.z=point.z;
		listener*/

		//TODO CONVERT INTO /MAP or /BASEFOOTPRING Frameid

		res.point.x=point.x;
		res.point.y=point.y;
		res.point.z=point.z;

		if(display_marker){
			visualization_msgs::MarkerArray m_array;
			int id=std::rand();
			addMarker(m_array,point.x,point.y,point.z,"test","/CameraTop_optical_frame",id);
			object_marker_pub.publish(m_array);
		}

		
		// ...
	} catch (...) {
    	return false;
	}
	
	return true;
}

void getDarkNetBoxesCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg){
	robocup_msgs::EntityList currentEntityList;
	
	//ROS_INFO("--> in   getDarkNetBoxesCallback");
	try{
		for(int i=0;i<msg->bounding_boxes.size();i++){
			darknet_ros_msgs::BoundingBox bounding_box=msg->bounding_boxes[i];
			if(!filter.compare(bounding_box.Class)){
				robocup_msgs::Entity currentEntity;
				//ROS_INFO("--> in LOOP  getDarkNetBoxesCallback");
				
				currentEntity.label=bounding_box.Class;
				//ROS_INFO("--> after Class  getDarkNetBoxesCallback");
				geometry_msgs::Pose currentPose;
				int rgb_w=(bounding_box.xmin + bounding_box.xmax) / 2;
				int rgb_h=(bounding_box.ymin + bounding_box.ymax) / 2;
				//ROS_INFO("--> after ymin  getDarkNetBoxesCallback");
				//process 2d ->3d
				std::string frame_id=msg->header.frame_id;
				currentEntity.header.frame_id=source_frame;
				int pcl_index;
				pcl::PointXYZ point;
				ROS_INFO("--> before ( (rgb_w*(current_cloud->width)) + rgb_h)  getDarkNetBoxesCallback");
				if (current_cloud != NULL){
					pcl_index = ( (rgb_w*(current_cloud->width)) + rgb_h);
					ROS_INFO("--> before fromROSMsg  getDarkNetBoxesCallback");
					pcl::fromROSMsg(*current_cloud,point_pcl);
					ROS_INFO("--> after fromROSMsg  getDarkNetBoxesCallback");
					point =point_pcl.at(pcl_index);
					if(isnan(point.x) || isnan(point.y) || isnan(point.z)){
							ROS_WARN("ISNAN value x:%f,y:%f,z:%f",point.x,point.y,point.z);
					}


					if(isinf(point.x) || isinf(point.y) || isinf(point.z)){
							ROS_WARN("ISINF value x:%f,y:%f,z:%f",point.x,point.y,point.z);
					}

					//TODO CONVERT INTO /MAP or /BASEFOOTPRING Frameid

					geometry_msgs::PointStamped pt_computed;
					pt_computed.header.frame_id=source_frame;
					pt_computed.header.stamp=ros::Time();
					pt_computed.point.x=point.x;
					pt_computed.point.y=point.y;
					pt_computed.point.z=point.z;

				
        			geometry_msgs::PointStamped map_point;
				
        			//need to transform coord from OPtical Frame to Base_Link or Map 
        			tf::TransformListener listener;
        			listener.waitForTransform(msg->header.frame_id, target_frame, ros::Time(0), ros::Duration(5));
        			try{
						listener.transformPoint(target_frame, pt_computed, map_point);
        				ROS_INFO("-->Pt a point optical ref : x:%f,y:%f",pt_computed.point.x,pt_computed.point.y);
        				ROS_INFO("-->Pt a point map ref : x:%f,y:%f",map_point.point.x,map_point.point.y);
        				pt_computed.point.x=map_point.point.x;
        				pt_computed.point.y=map_point.point.y;
        				pt_computed.header.frame_id = target_frame;
						currentPose.position.x=pt_computed.point.x;
						currentPose.position.y=pt_computed.point.y;
						currentPose.position.z=pt_computed.point.z;
						currentEntity.header.frame_id=target_frame;

    				}
    				catch(tf::TransformException& ex){
    					ROS_WARN("Received an exception trying to transform a point : %s", ex.what());
      				}
				}
				

				currentEntity.pose=currentPose;

				currentEntityList.entityList.push_back(currentEntity);
				//ROS_INFO("--> before display_marker  getDarkNetBoxesCallback");
				if(display_marker){
					visualization_msgs::MarkerArray m_array;
					int id=std::rand();
					addMarker(m_array,currentEntity.pose.position.x,currentEntity.pose.position.y,currentEntity.pose.position.z,"test",currentEntity.header.frame_id,id);
					object_marker_pub.publish(m_array);
				}
			}
		}

		// PUBLISH DATA
		object_entity_pub.publish(currentEntityList);
		ros::spinOnce();

	} catch (...) {
		ROS_WARN("Exception getDarkNetBoxesCallback...");
    	return ;
	}
}


/**
 * Main
 */
int main(int argc, char **argv) {

	// Initialize ros and create node handle
	ros::init(argc,argv,"convert_2d_to_3d");
	ros::NodeHandle nh;

	if (!ros::param::get("display_marker", display_marker))
    {
      display_marker=true;
    }


if (!ros::param::get("filter_class", filter))
    {
      filter="person";
    }

if (!ros::param::get("target_frame", target_frame))
    {
      target_frame="base_footprint";
    }

if (!ros::param::get("source_frame", source_frame))
    {
      source_frame="CameraTop_optical_frame";
    }


	


	// Publisher
	object_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/objet_pose_registred_pub", 100);
	object_entity_pub = nh.advertise<robocup_msgs::EntityList>("/objects/entity", 5);
	sub_registered_pcl=nh.subscribe("/pepper_robot/camera/depth_registered/points", 5, getPclCallback);
	//sub_registered_darknetBoxes=nh.subscribe("/darknet_ros/bounding_boxes",5, getDarkNetBoxesCallback);
	ros::ServiceServer service = nh.advertiseService("convert_2d_to_3d", convert2dto3dCallback);
   	ROS_INFO("Ready to convert rgb coord to pcl 3d point.");
    ros::spin();

}
