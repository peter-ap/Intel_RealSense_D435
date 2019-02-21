// include the librealsense C++ header file
#include <librealsense2/rs.hpp>

// include OpenCV header file
#include <opencv2/opencv.hpp>

// include pointcloud library for visualization
#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

using namespace std;
using namespace cv;

int main(int argc, char * argv[])
{ 
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0,0,0);

    // Declare pointcloud object, for calculating pointclouds and texture mappings
    rs2::pointcloud pc;
    // We want the points object to be persistent so we can display the last cloud when a frame drops
    rs2::points points;

    //Contruct a pipeline which abstracts the device
    rs2::pipeline pipe; 

    //Instruct pipeline to start streaming with the requested configuration
    pipe.start();

/*     // Camera warmup - dropping several first frames to let auto-exposure stabilize
    rs2::frameset frames;
    for (int i = 0; i < 30; i++)
{
	//Wait for all configured streams to produce a frame
	auto frames = pipe.wait_for_frames();
    auto depth = frames.get_depth_frame();
}
 */

while(!viewer->wasStopped()){
    //Wait for all configured streams to produce a frame
    auto frames = pipe.wait_for_frames();
    auto depth = frames.get_depth_frame();

    points = pc.calculate(depth);

    auto vertices = points.get_vertices();
    auto tex_coords = points.get_texture_coordinates();
    //Set points of depth
    
  cloud->points.resize (points.size()); 
    for (size_t i = 0; i < points.size(); ++i)
    {
        cloud->points[i].x = vertices[i].x;
        cloud->points[i].y = vertices[i].y;
        cloud->points[i].z = vertices[i].z;
    }

    //remove previous points
    viewer->removeAllPointClouds();
    //Add new points
    viewer->addPointCloud<pcl::PointXYZ>(cloud);

    viewer->spinOnce(0);
    boost::this_thread::sleep(boost::posix_time::microseconds(0));

}
    return 0; 


    }

