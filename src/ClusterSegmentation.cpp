/*

li
Author: Sean Cassero
7/15/15

Modified: Nikolas Tsagkopoulos
1/2021

*/


#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types_conversion.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <ros_assignment/SegmentedClustersArray.h>



class ClusterSegmentation {

public:

    explicit ClusterSegmentation(ros::NodeHandle nh) : m_nh(nh)  {

        ros::NodeHandle privateNodeHandle("~");
        std::string input_cloud_topic;
        if (!privateNodeHandle.getParam("input_cloud_topic", input_cloud_topic)) {
            ROS_ERROR("Could not find 'input_cloud_topic' param in param server.");
            exit(-1);
        }

        // define the subscriber and publisher
        m_sub = m_nh.subscribe(input_cloud_topic, 1, &ClusterSegmentation::cloudCallback, this);
        m_clusterPub = m_nh.advertise<ros_assignment::SegmentedClustersArray>("/cluster_segmentation/pcl_clusters", 1);
        pcl_pub_ = m_nh.advertise<sensor_msgs::PointCloud2>("/cluster_segmentation/segmented_clusters", 1);

	privateNodeHandle.param<bool>("debug", debug, false);
	if (debug) {
	    purple_mask_pub_ = m_nh.advertise<sensor_msgs::PointCloud2>("/cluster_segmentation/purple_mask", 1);
	}
    }

private:

    ros::NodeHandle m_nh;
    ros::Publisher m_pub;
    ros::Subscriber m_sub;
    ros::Publisher m_clusterPub;
    ros::Publisher pcl_pub_;
    ros::Publisher purple_mask_pub_;

    bool debug;

    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloudMsg);
    void publishPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud, std::string frameID);
    void publishPurpleMask(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud, std::string frameID);

}; // end class definition

void ClusterSegmentation::publishPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud, std::string frameID) {
    pcl::PCLPointCloud2 outputPCL;
    pcl::toPCLPointCloud2(*pointCloud, outputPCL);
    outputPCL.header.frame_id = frameID;
    pcl_pub_.publish(outputPCL);
}

void ClusterSegmentation::publishPurpleMask(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud, std::string frameID) {
    pcl::PCLPointCloud2 outputPCL;
    pcl::toPCLPointCloud2(*pointCloud, outputPCL);
    outputPCL.header.frame_id = frameID;
    purple_mask_pub_.publish(outputPCL);
}

// define callback function
void ClusterSegmentation::cloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloudMsg)
{

    ROS_INFO("Received cloud_msg");
    
    // Convert to PCL data type
    pcl::PCLPointCloud2Ptr cloudPtr(new pcl::PCLPointCloud2);
    pcl_conversions::toPCL(*cloudMsg, *cloudPtr);
    
    // Perform voxel grid downsampling filtering
    pcl::PCLPointCloud2Ptr cloudVoxelFilteredPtr(new pcl::PCLPointCloud2);
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (cloudPtr);
    sor.setLeafSize (0.01f, 0.01f, 0.01f);
    sor.filter (*cloudVoxelFilteredPtr);

    // Convert the pcl::PointCloud2Ptr type to pcl::PointCloud<pcl::PointXYZRGB>
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzCloudPtr(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromPCLPointCloud2(*cloudVoxelFilteredPtr, *xyzCloudPtr);
    //publishPointCloud(xyzCloudPtr, cloudMsg->header.frame_id);
    ROS_INFO("Applied voxel grid filter: %lu points", xyzCloudPtr->points.size());

    // Perform passthrough filtering
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzCloudPassFilteredPtr(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (xyzCloudPtr);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (.5, 2.5);
    //pass.setFilterLimitsNegative (true);
    pass.filter (*xyzCloudPassFilteredPtr);
    ROS_INFO("Applied passthrough filter: %lu points", xyzCloudPassFilteredPtr->points.size());

    // Convert the filtered pointcloud from RGB to HSV
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr xyzCloudHSV(new pcl::PointCloud<pcl::PointXYZHSV>);
    pcl::PointCloudXYZRGBtoXYZHSV(*xyzCloudPassFilteredPtr, *xyzCloudHSV);

    // Filter out the non purple values
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr xyzCloudHSVfiltered(new pcl::PointCloud<pcl::PointXYZHSV>);
    pcl::ConditionAnd<pcl::PointXYZHSV>::Ptr hRange(new pcl::ConditionAnd<pcl::PointXYZHSV> ());
    pcl::ConditionAnd<pcl::PointXYZHSV>::Ptr sRange(new pcl::ConditionAnd<pcl::PointXYZHSV> ());
    pcl::ConditionAnd<pcl::PointXYZHSV>::Ptr vRange(new pcl::ConditionAnd<pcl::PointXYZHSV> ());
    hRange->addComparison(pcl::FieldComparison<pcl::PointXYZHSV>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZHSV>("h", pcl::ComparisonOps::GT, 140.0)));
    hRange->addComparison(pcl::FieldComparison<pcl::PointXYZHSV>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZHSV>("h", pcl::ComparisonOps::LT, 260.0)));
    sRange->addComparison(pcl::FieldComparison<pcl::PointXYZHSV>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZHSV>("s", pcl::ComparisonOps::GT, 0.039)));
    sRange->addComparison(pcl::FieldComparison<pcl::PointXYZHSV>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZHSV>("s", pcl::ComparisonOps::LT, 1.0)));
    vRange->addComparison(pcl::FieldComparison<pcl::PointXYZHSV>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZHSV>("v", pcl::ComparisonOps::GT, 0.039)));
    vRange->addComparison(pcl::FieldComparison<pcl::PointXYZHSV>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZHSV>("v", pcl::ComparisonOps::LT, 1.0)));
    
    pcl::ConditionalRemoval<pcl::PointXYZHSV> hfilter;
    pcl::ConditionalRemoval<pcl::PointXYZHSV> sfilter;
    pcl::ConditionalRemoval<pcl::PointXYZHSV> vfilter;
    
    hfilter.setCondition (hRange);
    hfilter.setInputCloud (xyzCloudHSV);
    hfilter.setKeepOrganized (false);
    hfilter.filter (*xyzCloudHSVfiltered);
    
    sfilter.setCondition (sRange);
    sfilter.setInputCloud (xyzCloudHSVfiltered);
    sfilter.setKeepOrganized (false);
    sfilter.filter (*xyzCloudHSVfiltered);
    
    vfilter.setCondition (vRange);
    vfilter.setInputCloud (xyzCloudHSVfiltered);
    vfilter.setKeepOrganized (false);
    vfilter.filter (*xyzCloudHSVfiltered);
    
    ROS_INFO("Applied HSV purple filter: %lu points", xyzCloudHSVfiltered->points.size());
    
    // Convert the HSV Filtered pointcloud back to RGB
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzCloudRGBfiltered(new pcl::PointCloud<pcl::PointXYZRGB>);
    xyzCloudRGBfiltered->width = xyzCloudHSVfiltered->width;
    xyzCloudRGBfiltered->height = xyzCloudHSVfiltered->height;
    xyzCloudRGBfiltered->points.resize (xyzCloudRGBfiltered->width * xyzCloudRGBfiltered->height);

    for (int i=0; i<xyzCloudHSVfiltered->points.size(); i++) {
    	pcl::PointXYZHSVtoXYZRGB(xyzCloudHSVfiltered->points[i], xyzCloudRGBfiltered->points[i]);
    }

    if (debug) {
	publishPurpleMask(xyzCloudRGBfiltered, cloudMsg->header.frame_id);
    }

    // create a pcl object to hold the ransac filtered results
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzCloudPtrRansacFiltered(new pcl::PointCloud<pcl::PointXYZRGB>);

    // perform ransac planar filtration to remove table top
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGB> seg1;
    // Optional
    seg1.setOptimizeCoefficients(true);
    // Mandatory
    seg1.setModelType (pcl::SACMODEL_PLANE);
    seg1.setMethodType (pcl::SAC_RANSAC);
    seg1.setDistanceThreshold(0.04);

    seg1.setInputCloud (xyzCloudRGBfiltered);
    seg1.segment (*inliers, *coefficients);

    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;

    //extract.setInputCloud (xyzCloudPtrFiltered);
    extract.setInputCloud(xyzCloudRGBfiltered);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*xyzCloudPtrRansacFiltered);

//    publishPointCloud(xyzCloudPtrRansacFiltered, cloudMsg->header.frame_id);
    ROS_INFO("Applied RANSAC filter");

    // perform euclidean cluster segmentation to seporate individual objects

    // Create the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud (xyzCloudPtrRansacFiltered);

    // create the extraction object for the clusters
    std::vector<pcl::PointIndices> cluster_indices_list;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    // specify euclidean cluster parameters
    ec.setClusterTolerance (0.02); // 2cm
    ec.setMinClusterSize (100);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(xyzCloudPtrRansacFiltered);
    // exctract the indices pertaining to each cluster and store in a vector of pcl::PointIndices
    ec.extract(cluster_indices_list);

    // declare an instance of the SegmentedClustersArray message
    ros_assignment::SegmentedClustersArray cloudClusters;

    // declare the output variable instances
    sensor_msgs::PointCloud2 output;
    pcl::PCLPointCloud2 outputPCL;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr concatenatedClusters(new pcl::PointCloud<pcl::PointXYZRGB>);

    // here, cluster_indices is a vector of indices for each cluster. iterate through each indices object to work with them seporately
    for (const pcl::PointIndices &cluster_indices : cluster_indices_list)
    {
        // create a pcl object to hold the extracted cluster
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr clusterPtr(new pcl::PointCloud<pcl::PointXYZRGB>);

        // now we are in a vector of indices pertaining to a single cluster.
        // Assign each point corresponding to this cluster in xyzCloudPtrPassthroughFiltered a specific color for identification purposes
        int red = rand() % 256;
        int green = rand() % 256;
        int blue = rand() % 256;

	for (const int index : cluster_indices.indices)
        {
	    xyzCloudPtrRansacFiltered->points[index].r = red;
	    xyzCloudPtrRansacFiltered->points[index].g = green;
	    xyzCloudPtrRansacFiltered->points[index].b = blue;
            clusterPtr->points.push_back(xyzCloudPtrRansacFiltered->points[index]);
        }

        // convert to pcl::PCLPointCloud2
        pcl::toPCLPointCloud2(*clusterPtr, outputPCL);

        // Convert to ROS data type
        pcl_conversions::fromPCL(outputPCL, output);

        concatenatedClusters->operator+=(*clusterPtr);

        // add the cluster to the array message
        cloudClusters.clusters.push_back(output);
    }

    publishPointCloud(concatenatedClusters, cloudMsg->header.frame_id);

    cloudClusters.header.frame_id = cloudMsg->header.frame_id;

    // publish the clusters
    m_clusterPub.publish(cloudClusters);
}


int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "cluster_segmentation");
    ros::NodeHandle nh;

    ClusterSegmentation segs(nh);

    while(ros::ok())
        ros::spin();
}
