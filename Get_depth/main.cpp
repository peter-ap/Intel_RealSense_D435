// include the librealsense C++ header file
#include <librealsense2/rs.hpp>

// include OpenCV header file
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

int main()
{

    // Declare pointcloud object, for calculating pointclouds and texture mappings
    rs2::pointcloud pc;
    // We want the points object to be persistent so we can display the last cloud when a frame drops
    rs2::points points;

    //Contruct a pipeline which abstracts the device
    rs2::pipeline pipe;

    //Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;

    //Add desired streams to configuration
    cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_BGR8, 30);
    cfg.enable_stream(RS2_STREAM_INFRARED, 1280, 720,RS2_FORMAT_Y8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH,1280, 720,RS2_FORMAT_Z16,30);
    //Instruct pipeline to start streaming with the requested configuration
    pipe.start(cfg);

    // Camera warmup - dropping several first frames to let auto-exposure stabilize
    rs2::frameset frames;
    for (int i = 0; i < 30; i++)
{
	//Wait for all configured streams to produce a frame
	auto frames = pipe.wait_for_frames();
    auto depth = frames.get_depth_frame();
}
while(1){
    //Wait for all configured streams to produce a frame
    auto frames = pipe.wait_for_frames();
    auto depth = frames.get_depth_frame();


    //Generate the pointcloud and texture mappings
    points = pc.calculate(depth);
    
    auto vertices = points.get_vertices();
    auto tex_coords = points.get_texture_coordinates();
    //cout << points.size() << endl;
    
    for (int i = 0; i<points.size();i++){
        float x = vertices[i].x;
        float y = vertices[i].y;
        float z = vertices[i].z;
        cout << x << y << z << endl;
    }
    //Get each frame
    auto color_frame = frames.get_color_frame();
    auto infrared_frame=frames.get_infrared_frame();

    // Creating OpenCV Matrix from a color image
    Mat color(Size(1280, 720), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);
    Mat infrared(Size(1280, 720), CV_8UC1, (void*)infrared_frame.get_data(), Mat::AUTO_STEP);

    // Display in a GUI
    //namedWindow("Display Image", WINDOW_AUTOSIZE );
    imshow("Display Image", color);
    imshow("Display_Infrared_Image", infrared);
    if( waitKey(10) == 27 ) break; // stop capturing by pressing ESC 
    }

    return 0;
}