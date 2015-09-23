#include <iostream>
#include <boost/filesystem.hpp> // Check if files exist.
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>
#include <time.h>
#include <ctime> // For tic toc.

// Input / Output header files.
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/pcd_io.h>

// For creating the origin cloud.
#include <pcl/point_types.h>

// For rotation/translation
#include <pcl/common/transforms.h>

// Visualizer header files.
#include <pcl/visualization/pcl_visualizer.h>

// Removing unused vertices
#include <pcl/surface/simplification_remove_unused_vertices.h>


using namespace std;

//#define ACTUAL_VIEW
//#define VIRTUAL_VIEW
//#define VIRTUAL_VIEW_BACKFACE_AND_FRUSTUM_CULLING
//#define VIRTUAL_VIEW_INTERSECTED_POINTS
//#define VIRTUAL_VIEW_SELECTED_POINTS
#define VIRTUAL_VIEW_SELECTED_NOISY_POINTS


float pointCountSample(float x, std::string sample_name)
{
    if(sample_name == "white")
    {
        float p1 = 0.00007629;
        float p2 = -0.0002461;
        float p3 = -0.8901;
        float p4 = 1.343;
        float p5 = 2968;

        float pointCountOutput = p1*pow(x,4) + p2*pow(x,3) + p3*pow(x,2) + p4*x + p5;

        return pointCountOutput;
    }
    else if(sample_name == "10")
    {
        float a1 =        2662;
        float b1 =     0.02081;
        float c1 =       1.559;
        float a2 =       282.1;
        float b2 =       0.134;
        float c2 =       -1.74;
        float a3 =       254.5;
        float b3 =      0.2276;
        float c3 =      -1.874;
        float a4 =       135.2;
        float b4 =      0.4265;
        float c4 =      -2.168;
        float a5 =       197.4;
        float b5 =      0.3263;
        float c5 =      -2.033;
        float a6 =       86.56;
        float b6 =      0.5273;
        float c6 =      -2.289;
        float a7 =       48.85;
        float b7 =      0.6264;
        float c7 =      -2.364;

        float pointCountOutput = a1*sin((b1*x+c1)*M_PI/180) + a2*sin((b2*x+c2)*M_PI/180) + a3*sin((b3*x+c3)*M_PI/180) +
                                 a4*sin((b4*x+c4)*M_PI/180) + a5*sin((b5*x+c5)*M_PI/180) + a6*sin((b6*x+c6)*M_PI/180) +
                                 a7*sin((b7*x+c7)*M_PI/180);

        return pointCountOutput;
    }
    else if(sample_name == "11")
    {
        float a1 =        4664;
        float b1 =      0.0283;
        float c1 =       1.594;
        float a2 =       535.4;
        float b2 =      0.1257;
        float c2 =      -1.944;
        float a3 =       69.65;
        float b3 =      0.1917;
        float c3 =      0.0745;
        float a4 =       293.8;
        float b4 =      0.3017;
        float c4 =      -2.256;
        float a5 =        2066;
        float b5 =     0.03184;
        float c5 =      -1.468;
        float a6 =       332.4;
        float b6 =      0.2138;
        float c6 =      -2.076;
        float a7 =       92.75;
        float b7 =       0.491;
        float c7 =      -2.554;
        float a8 =       178.7;
        float b8 =      0.3954;
        float c8 =      -2.327;

        float pointCountOutput = a1*sin((b1*x+c1)*M_PI/180) + a2*sin((b2*x+c2)*M_PI/180) + a3*sin((b3*x+c3)*M_PI/180) +
                                 a4*sin((b4*x+c4)*M_PI/180) + a5*sin((b5*x+c5)*M_PI/180) + a6*sin((b6*x+c6)*M_PI/180) +
                                 a7*sin((b7*x+c7)*M_PI/180) + a8*sin((b8*x+c8)*M_PI/180);

        return pointCountOutput;
    }
    else if(sample_name == "12")
    {
        float a1 =        2409;
        float b1 =      0.0371;
        float c1 =       1.567;
        float a2 =       561.1;
        float b2 =      0.1234;
        float c2 =       1.545;
        float a3 =       459.4;
        float b3 =      0.1804;
        float c3 =       4.501;
        float a4 =       222.9;
        float b4 =      0.2726;
        float c4 =      -1.719;
        float a5 =       79.81;
        float b5 =       0.393;
        float c5 =      -2.228;
        float a6 =       83.85;
        float b6 =      0.4911;
        float c6 =      -2.356;
        float a7 =       49.02;
        float b7 =      0.6803;
        float c7 =      -3.163;
        float a8 =       42.91;
        float b8 =      0.5886;
        float c8 =      -2.828;

        float pointCountOutput = a1*sin((b1*x+c1)*M_PI/180) + a2*sin((b2*x+c2)*M_PI/180) + a3*sin((b3*x+c3)*M_PI/180) +
                                 a4*sin((b4*x+c4)*M_PI/180) + a5*sin((b5*x+c5)*M_PI/180) + a6*sin((b6*x+c6)*M_PI/180) +
                                 a7*sin((b7*x+c7)*M_PI/180) + a8*sin((b8*x+c8)*M_PI/180);

        return pointCountOutput;
    }
    else
    {
        cout << "\033[1;31m ********************************************\033[0m" << endl;
        cout << "\033[1;31m ********************************************\033[0m" << endl;
        cout << "\033[1;31m !!!!sampleSelector is an invalid value!!!!!!\033[0m" << endl;
        cout << "\033[1;31m ********************************************\033[0m" << endl;
        cout << "\033[1;31m ********************************************\033[0m" << endl;
        exit(EXIT_FAILURE);
    }
}

float dotProductAngle(pcl::PointXYZ p1, pcl::PointXYZ p2)
{
    float p1p2dot = (p1.x * p2.x + p1.y * p2.y + p1.z * p2.z);
    float p1Magnitude = sqrt(pow(p1.x,2) + pow(p1.y,2) + pow(p1.z,2));
    float p2Magnitude = sqrt(pow(p2.x,2) + pow(p2.y,2) + pow(p2.z,2));
    float p1p2angle = acos(p1p2dot / (p1Magnitude*p2Magnitude));

    return p1p2angle;   // In radians
}

int main()
{
    // Temporary Variables - For testing purposes only, shouldn't be here in release code.
    /// Future Changes Required:
    /// -Number of points layed needs to vary with camera distance from the object, further away less points etc.

//    cout << norm_rnd() << endl;
//    cout << norm_rnd() << endl;
//    cout << norm_rnd() << endl;
//    cout << norm_rnd() << endl;
//    cout << norm_rnd() << endl;
//    cout << norm_rnd() << endl;
//    cout << norm_rnd() << endl;
//    cout << norm_rnd() << endl;
//    cout << norm_rnd() << endl;

    time_t tstart,tend;
    tstart = time(0);
    //////////////////////////// Initialize the variables /////////////////////////////
    pcl::PolygonMesh objectMesh;

    double pixelGridDistance = 2;

    // Camera roll,pitch and yaw - relative to WCS
    float cameraRollZAxis = 0*M_PI/180; //
    float cameraPitchXAxis = (90-54.7)*M_PI/180; // 45
    float cameraYawYAxis = -135*M_PI/180; //-135
    float cameraXDistance = 0.346;
    float cameraYDistance = 0.346;
    float cameraZDistance = 0.346;

    // Camera Specfic Values
    double cameraYFOVRadians = 0.38008945; //21.7775213deg //Angle represents half the field of view.
    double cameraXFOVRadians = 0.49807725; //28.5377241deg //Angle represents half the field of view.
    double cameraVerticalPixels = 480;
    double cameraHorizontalPixels = 752;

    // Sample Specific
    std::string sampleSelector = "white";

    // Initialize normal distribution values;
    boost::mt19937 generator;
    generator.seed(time(0));
    boost::normal_distribution<> norm_dist(0.002,0.001);
    boost::variate_generator<boost::mt19937&,boost::normal_distribution<> > norm_rnd(generator, norm_dist);
    ///////////////////////////////////////////////////////////////////////////////////

    /////////////////////////////////// Load files ////////////////////////////////////
    pcl::io::load("./Models/sphere_0-05_2520.ply", objectMesh);
    pcl::PointCloud<pcl::PointXYZ>::Ptr objectPointCloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointNormal>::Ptr objectNormalCloud (new pcl::PointCloud<pcl::PointNormal>);

    // Extract Point and Normal cloud from the mesh file. Storing them seperatley.
    pcl::fromPCLPointCloud2(objectMesh.cloud,*objectPointCloud);
    pcl::fromPCLPointCloud2(objectMesh.cloud,*objectNormalCloud); // Will throw a terminal warning if object_mesh doesn't contain curvature data. Can be ignored.

    // Check to see if the objectNormalCloud has normals, extracted from objectMesh.
    if(objectNormalCloud->at(0).normal_x == 0 && objectNormalCloud->at(0).normal_y == 0 && objectNormalCloud->at(0).normal_z == 0 )
    {
        cout << "\033[1;31m ********************************************\033[0m" << endl;
        cout << "\033[1;31m ********************************************\033[0m" << endl;
        cout << "\033[1;31m !!Check for Normals in the objectMesh file!!\033[0m" << endl;
        cout << "\033[1;31m !!!Backface culling may have been skipped!!!\033[0m" << endl;
        cout << "\033[1;31m ********************************************\033[0m" << endl;
        cout << "\033[1;31m ********************************************\033[0m" << endl;
    }

    ///////////////////////////////////////////////////////////////////////////////////

    ////////////////////////// Create the camera point cloud //////////////////////////
    pcl::PointCloud<pcl::PointXYZ>::Ptr cameraCloudOne (new pcl::PointCloud<pcl::PointXYZ>);
    // If cameraCloudOne.pcd doesn't exist, create the point cloud. Else, load camera.pcd.   Note: cameraCloudOne.pcd has to be contained in the build folder.
    if (!boost::filesystem::exists("./Models/cameraCloudOne.pcd"))
    {
        cameraCloudOne->width = 1;
        cameraCloudOne->height = 9;
        cameraCloudOne->is_dense = false;
        cameraCloudOne->points.resize(cameraCloudOne->width * cameraCloudOne->height);

        cameraCloudOne->points[0].x = 0.0;
        cameraCloudOne->points[0].y = 0.0;
        cameraCloudOne->points[0].z = 0.0;

        cameraCloudOne->points[1].x = 0.025;
        cameraCloudOne->points[1].y = 0.01;
        cameraCloudOne->points[1].z = 0.0;

        cameraCloudOne->points[2].x = -0.025;
        cameraCloudOne->points[2].y = 0.01;
        cameraCloudOne->points[2].z = 0.0;

        cameraCloudOne->points[3].x = 0.025;
        cameraCloudOne->points[3].y = -0.01;
        cameraCloudOne->points[3].z = 0.0;

        cameraCloudOne->points[4].x = -0.025;
        cameraCloudOne->points[4].y = -0.01;
        cameraCloudOne->points[4].z = 0.0;

        cameraCloudOne->points[5].x = 0.0;
        cameraCloudOne->points[5].y = 0.0;
        cameraCloudOne->points[5].z = 0.01;

        cameraCloudOne->points[6].x = 0.0;
        cameraCloudOne->points[6].y = 0.0;
        cameraCloudOne->points[6].z = 0.025;

        cameraCloudOne->points[7].x = 0.0;
        cameraCloudOne->points[7].y = 0.0;
        cameraCloudOne->points[7].z = -0.01;

        cameraCloudOne->points[8].x = 0.0;
        cameraCloudOne->points[8].y = 0.01;
        cameraCloudOne->points[8].z = 0.0;

        pcl::io::savePCDFile("./Models/cameraCloudOne.pcd", *cameraCloudOne);
    }
    else
    {
        pcl::io::loadPCDFile("./Models/cameraCloudOne.pcd", *cameraCloudOne);
    }
    ///////////////////////////////////////////////////////////////////////////////////


    /////////////////////////// Transform creation and use ////////////////////////////
    // Create a new point cloud variable for transformed workspace view.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cameraCloudOneActualView (new pcl::PointCloud<pcl::PointXYZ>);

    /// Translate and Rotate Object
        Eigen::Matrix4f actualViewTransform = Eigen::Matrix4f::Identity();

        // Rotation section
        actualViewTransform(0,0) = cos(cameraRollZAxis)*cos(cameraYawYAxis);
        actualViewTransform(0,1) = (cos(cameraRollZAxis)*sin(cameraYawYAxis)*sin(cameraPitchXAxis)) - (sin(cameraRollZAxis)*cos(cameraPitchXAxis));
        actualViewTransform(0,2) = (cos(cameraRollZAxis)*sin(cameraYawYAxis)*cos(cameraPitchXAxis)) + (sin(cameraRollZAxis)*sin(cameraPitchXAxis));
        actualViewTransform(1,0) = sin(cameraRollZAxis)*cos(cameraYawYAxis);
        actualViewTransform(1,1) = (sin(cameraRollZAxis)*sin(cameraYawYAxis)*sin(cameraPitchXAxis)) + (cos(cameraRollZAxis)*cos(cameraPitchXAxis));
        actualViewTransform(1,2) = (sin(cameraRollZAxis)*sin(cameraYawYAxis)*cos(cameraPitchXAxis)) - (cos(cameraRollZAxis)*sin(cameraPitchXAxis));
        actualViewTransform(2,0) = -sin(cameraYawYAxis);
        actualViewTransform(2,1) = sin(cameraPitchXAxis)*cos(cameraYawYAxis);
        actualViewTransform(2,2) = cos(cameraPitchXAxis)*cos(cameraYawYAxis);

        // Translation section
        actualViewTransform(0,3) = cameraXDistance; //x
        actualViewTransform(1,3) = cameraYDistance; //y
        actualViewTransform(2,3) = cameraZDistance; //z

        pcl::transformPointCloud (*cameraCloudOne, *cameraCloudOneActualView, actualViewTransform);
    ///

#ifdef ACTUAL_VIEW
    // Create the visualizer
    pcl::visualization::PCLVisualizer viewerOne ("Visualize the actual scene");

    // Add objectPointCLoudWorkspaceView and cameraCloudOne
    viewerOne.addPolygonMesh(objectMesh, "objectMesh");

    // Create a color handler for the origin point cloud - R,G,B colors
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cameraColorHandlerActualView (cameraCloudOneActualView, 0, 0, 255);

    // Add the point cloud to the viewer and pass the color handler
    viewerOne.addPointCloud(cameraCloudOneActualView, cameraColorHandlerActualView, "ActualViewCameraOne");

    // Set size of the origin point cloud
    viewerOne.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "ActualViewCameraOne");

    // Draw arrows to indicate the correct lengths have been drawn to help the user.
    std::basic_string<char> arrowXDistanceNameActualView = "arrowX";
    std::basic_string<char> arrowYDistanceNameActualView = "arrowY";
    std::basic_string<char> arrowZDistanceNameActualView = "arrowZ";

    // Add functionality to enable/disable distance if required.
    viewerOne.addArrow<pcl::PointXYZ,pcl::PointXYZ>(pcl::PointXYZ(0,cameraYDistance,cameraZDistance),pcl::PointXYZ(cameraXDistance,cameraYDistance,cameraZDistance),1.0,0.0,0.0,arrowXDistanceNameActualView,0);
    viewerOne.addArrow<pcl::PointXYZ,pcl::PointXYZ>(pcl::PointXYZ(cameraXDistance,0,cameraZDistance),pcl::PointXYZ(cameraXDistance,cameraYDistance,cameraZDistance),1.0,0.0,0.0,arrowYDistanceNameActualView,0);
    viewerOne.addArrow<pcl::PointXYZ,pcl::PointXYZ>(pcl::PointXYZ(cameraXDistance,cameraYDistance,0),pcl::PointXYZ(cameraXDistance,cameraYDistance,cameraZDistance),1.0,0.0,0.0,arrowZDistanceNameActualView,0);

    // Add label to origin for x,y,z point position.
    pcl::PointXYZ xTextActualView;
    xTextActualView.x = 2.0;
    xTextActualView.y = 0;
    xTextActualView.z = 0;
    viewerOne.addText3D("+X",xTextActualView,0.1,1.0,1.0,1.0,"x",0);
    viewerOne.addLine(pcl::PointXYZ(0,0,0),pcl::PointXYZ(2,0,0),"line_x",0);

    pcl::PointXYZ yTextActualView;
    yTextActualView.x = 0;
    yTextActualView.y = 2.0;
    yTextActualView.z = 0;
    viewerOne.addText3D("+Y",yTextActualView,0.1,1.0,1.0,1.0,"y",0);
    viewerOne.addLine(pcl::PointXYZ(0,0,0),pcl::PointXYZ(0,2,0),"line_y",0);

    pcl::PointXYZ zTextActualView;
    zTextActualView.x = 0;
    zTextActualView.y = 0;
    zTextActualView.z = 2.0;
    viewerOne.addText3D("+Z",zTextActualView,0.1,1.0,1.0,1.0,"z",0);
    viewerOne.addLine(pcl::PointXYZ(0,0,0),pcl::PointXYZ(0,0,2),"line_z",0);

    viewerOne.setCameraClipDistances(5.0,20.0);
    viewerOne.setCameraPosition(0,0,-10, 0,1,0, 0);

    // Display the visualiser until 'q' key is pressed
    while (!viewerOne.wasStopped ()) {
      viewerOne.spinOnce ();
    }
    viewerOne.close();
    ///////////////////////////////////////////////////////////////////////////////////
#endif

    /////////////////////////// Transform creation and use ////////////////////////////
    /// Translate Object
        Eigen::Matrix4f virtualViewTranslation = Eigen::Matrix4f::Identity();

        // Translation section
        virtualViewTranslation(0,3) = -cameraXDistance;
        virtualViewTranslation(1,3) = -cameraYDistance;
        virtualViewTranslation(2,3) = -cameraZDistance;

        pcl::transformPointCloud (*objectPointCloud, *objectPointCloud, virtualViewTranslation);
    ///
    /// Rotate Object Yaw
        Eigen::Matrix4f virtualViewRotationYawYAxis = Eigen::Matrix4f::Identity();

        // Rotation section
        virtualViewRotationYawYAxis(0,0) = cos(0.0)*cos(-cameraYawYAxis);
        virtualViewRotationYawYAxis(0,1) = (cos(0.0)*sin(-cameraYawYAxis)*sin(0.0)) - (sin(0.0)*cos(0.0));
        virtualViewRotationYawYAxis(0,2) = (cos(0.0)*sin(-cameraYawYAxis)*cos(0.0)) + (sin(0.0)*sin(0.0));
        virtualViewRotationYawYAxis(1,0) = sin(0.0)*cos(-cameraYawYAxis);
        virtualViewRotationYawYAxis(1,1) = (sin(0.0)*sin(-cameraYawYAxis)*sin(0.0)) + (cos(0.0)*cos(0.0));
        virtualViewRotationYawYAxis(1,2) = (sin(0.0)*sin(-cameraYawYAxis)*cos(0.0)) - (cos(0.0)*sin(0.0));
        virtualViewRotationYawYAxis(2,0) = -sin(-cameraYawYAxis);
        virtualViewRotationYawYAxis(2,1) = sin(0.0)*cos(-cameraYawYAxis);
        virtualViewRotationYawYAxis(2,2) = cos(0.0)*cos(-cameraYawYAxis);

        pcl::transformPointCloud (*objectPointCloud, *objectPointCloud, virtualViewRotationYawYAxis);
        pcl::transformPointCloud (*objectNormalCloud, *objectNormalCloud, virtualViewRotationYawYAxis);
    ///
    /// Rotate Object Pitch
        Eigen::Matrix4f virtualViewRotationPitchXAxis = Eigen::Matrix4f::Identity();

        // Rotation section
        virtualViewRotationPitchXAxis(0,0) = cos(0.0)*cos(0.0);
        virtualViewRotationPitchXAxis(0,1) = (cos(0.0)*sin(0.0)*sin(-cameraPitchXAxis)) - (sin(0.0)*cos(-cameraPitchXAxis));
        virtualViewRotationPitchXAxis(0,2) = (cos(0.0)*sin(0.0)*cos(-cameraPitchXAxis)) + (sin(0.0)*sin(-cameraPitchXAxis));
        virtualViewRotationPitchXAxis(1,0) = sin(0.0)*cos(0.0);
        virtualViewRotationPitchXAxis(1,1) = (sin(0.0)*sin(0.0)*sin(-cameraPitchXAxis)) + (cos(0.0)*cos(-cameraPitchXAxis));
        virtualViewRotationPitchXAxis(1,2) = (sin(0.0)*sin(0.0)*cos(-cameraPitchXAxis)) - (cos(0.0)*sin(-cameraPitchXAxis));
        virtualViewRotationPitchXAxis(2,0) = -sin(0.0);
        virtualViewRotationPitchXAxis(2,1) = sin(-cameraPitchXAxis)*cos(0.0);
        virtualViewRotationPitchXAxis(2,2) = cos(-cameraPitchXAxis)*cos(0.0);

        pcl::transformPointCloud (*objectPointCloud, *objectPointCloud, virtualViewRotationPitchXAxis);
        pcl::transformPointCloud (*objectNormalCloud, *objectNormalCloud, virtualViewRotationPitchXAxis);
    ///
    /// Rotate Object Roll
        Eigen::Matrix4f virtualViewRotationRollZAxis = Eigen::Matrix4f::Identity();

        // Rotation section
        virtualViewRotationRollZAxis(0,0) = cos(-cameraRollZAxis)*cos(0.0);
        virtualViewRotationRollZAxis(0,1) = (cos(-cameraRollZAxis)*sin(0.0)*sin(0.0)) - (sin(-cameraRollZAxis)*cos(0.0));
        virtualViewRotationRollZAxis(0,2) = (cos(-cameraRollZAxis)*sin(0.0)*cos(0.0)) + (sin(-cameraRollZAxis)*sin(0.0));
        virtualViewRotationRollZAxis(1,0) = sin(-cameraRollZAxis)*cos(0.0);
        virtualViewRotationRollZAxis(1,1) = (sin(-cameraRollZAxis)*sin(0.0)*sin(0.0)) + (cos(-cameraRollZAxis)*cos(0.0));
        virtualViewRotationRollZAxis(1,2) = (sin(-cameraRollZAxis)*sin(0.0)*cos(0.0)) - (cos(-cameraRollZAxis)*sin(0.0));
        virtualViewRotationRollZAxis(2,0) = -sin(0.0);
        virtualViewRotationRollZAxis(2,1) = sin(0.0)*cos(0.0);
        virtualViewRotationRollZAxis(2,2) = cos(0.0)*cos(0.0);

        pcl::transformPointCloud (*objectPointCloud, *objectPointCloud, virtualViewRotationRollZAxis);
        pcl::transformPointCloud (*objectNormalCloud, *objectNormalCloud, virtualViewRotationRollZAxis);
    ///

        // Put transformed cloud data back into the object_mesh
        pcl::toPCLPointCloud2(*objectPointCloud,objectMesh.cloud);

    //////////////////////////// Create the virtual plane /////////////////////////////
    pcl::PointCloud<pcl::PointXYZ>::Ptr pixelGridCloud (new pcl::PointCloud<pcl::PointXYZ>);
    pixelGridCloud->width = cameraHorizontalPixels;
    pixelGridCloud->height = cameraVerticalPixels;
    pixelGridCloud->is_dense = false;
    pixelGridCloud->points.resize(pixelGridCloud->width * pixelGridCloud->height); //360960 pixels/points

    // Calculate plane horizontal and vertical widths
    double pixelGridHorizontalWidthHalf = (pixelGridDistance*tan(cameraXFOVRadians));
    double pixelGridVerticalWidthHalf = (pixelGridDistance*tan(cameraYFOVRadians));

    // Start to drop points/pixels across plane surface.
    int i = 0;
    // Start from the bottom of the plane (wrt wcs as Z+ upwards) and add distance between each pixel until the top of the plane is reached.
    for (double vertical = -pixelGridVerticalWidthHalf; vertical < pixelGridVerticalWidthHalf; vertical = vertical + ((2*pixelGridVerticalWidthHalf)/cameraVerticalPixels))
    {
        // Start from the right of the plane (wrt wcs as Y+ to left) and add distance between each pixel until the left of the plane is reached.
        for (double horizontal = -pixelGridHorizontalWidthHalf; horizontal < pixelGridHorizontalWidthHalf; horizontal = horizontal + ((2*pixelGridHorizontalWidthHalf)/cameraHorizontalPixels))
        {
            pixelGridCloud->points[i].x = horizontal;
            pixelGridCloud->points[i].y = vertical;
            pixelGridCloud->points[i].z = pixelGridDistance;
            i++;
        }
    }

    // Define locations for each corner of the frustum.
    pcl::PointXYZ frustumBottomRight;
    frustumBottomRight.x = -pixelGridHorizontalWidthHalf;
    frustumBottomRight.y = -pixelGridVerticalWidthHalf;
    frustumBottomRight.z = pixelGridDistance;

    pcl::PointXYZ frustumTopRight;
    frustumTopRight.x = -pixelGridHorizontalWidthHalf;
    frustumTopRight.y = pixelGridVerticalWidthHalf-((2*pixelGridVerticalWidthHalf)/cameraVerticalPixels);
    frustumTopRight.z = pixelGridDistance;

    pcl::PointXYZ frustumBottomLeft;
    frustumBottomLeft.x = pixelGridHorizontalWidthHalf-((2*pixelGridHorizontalWidthHalf)/cameraHorizontalPixels);
    frustumBottomLeft.y = -pixelGridVerticalWidthHalf;
    frustumBottomLeft.z = pixelGridDistance;

    pcl::PointXYZ frustumTopLeft;
    frustumTopLeft.x = pixelGridHorizontalWidthHalf-((2*pixelGridHorizontalWidthHalf)/cameraHorizontalPixels);
    frustumTopLeft.y = pixelGridVerticalWidthHalf-((2*pixelGridVerticalWidthHalf)/cameraVerticalPixels);
    frustumTopLeft.z = pixelGridDistance;

    pcl::io::savePCDFile("./Models/pixelGridCloud.pcd", *pixelGridCloud);
    ///////////////////////////////////////////////////////////////////////////////////
    // Global visualizer values
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cameraColorHandlerVirtualView (cameraCloudOne, 0, 0, 255);
    pcl::PointXYZ xTextVirtualView;
    xTextVirtualView.x = 2.0;
    xTextVirtualView.y = 0;
    xTextVirtualView.z = 0;
    pcl::PointXYZ yTextVirtualView;
    yTextVirtualView.x = 0;
    yTextVirtualView.y = 2.0;
    yTextVirtualView.z = 0;
    pcl::PointXYZ zTextVirtualView;
    zTextVirtualView.x = 0;
    zTextVirtualView.y = 0;
    zTextVirtualView.z = 2.0;
    //
#ifdef VIRTUAL_VIEW

    // Create the visualizer
    pcl::visualization::PCLVisualizer viewerTwo ("Visualize the actual scene");
    // Add objectPointCLoudWorkspaceView and cameraCloudOne
    viewerTwo.addPolygonMesh(objectMesh, "objectMesh");

    // Create a color handler for the origin point cloud - R,G,B colors
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cameraColorHandlerVirtualView (cameraCloudOne, 0, 0, 255);

    // Add the point cloud to the viewer and pass the color handler
    viewerTwo.addPointCloud(cameraCloudOne, cameraColorHandlerVirtualView, "VirtualCameraOne");

    // Set size of the origin point cloud
    viewerTwo.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "VirtualCameraOne");

    /// Add pixelGridCloud Points ///
    // Create a color handler for the origin point cloud - R,G,B colors
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> pixelGridColorHandler (pixelGridCloud, 0, 255,0);

    //Represent pixelGridCloud
    //viewerTwo.addPointCloud(pixelGridCloud, pixelGridColorHandler, "pixelGridCloud");
    viewerTwo.addLine(frustumBottomRight,frustumTopRight,"rightSide",0);
    viewerTwo.addLine(frustumTopRight,frustumTopLeft,"topSide",0);
    viewerTwo.addLine(frustumTopLeft,frustumBottomLeft,"leftSide",0);
    viewerTwo.addLine(frustumBottomLeft,frustumBottomRight,"bottomSide",0);

    //Draw frustum itself.
    viewerTwo.addLine(pcl::PointXYZ(0,0,0),frustumTopRight,"topRight",0);
    viewerTwo.addLine(pcl::PointXYZ(0,0,0),frustumTopLeft,"topLeft",0);
    viewerTwo.addLine(pcl::PointXYZ(0,0,0),frustumBottomLeft,"bottomLeft",0);
    viewerTwo.addLine(pcl::PointXYZ(0,0,0),frustumBottomRight,"bottomRight",0);

    // Add label to origin for x,y,z point position.

    viewerTwo.addText3D("+X",xTextVirtualView,0.1,1.0,1.0,1.0,"x",0);
    viewerTwo.addLine(pcl::PointXYZ(0,0,0),pcl::PointXYZ(2,0,0),"line_x",0);


    viewerTwo.addText3D("+Y",yTextVirtualView,0.1,1.0,1.0,1.0,"y",0);
    viewerTwo.addLine(pcl::PointXYZ(0,0,0),pcl::PointXYZ(0,2,0),"line_y",0);


    viewerTwo.addText3D("+Z",zTextVirtualView,0.1,1.0,1.0,1.0,"z",0);
    viewerTwo.addLine(pcl::PointXYZ(0,0,0),pcl::PointXYZ(0,0,2),"line_z",0);

    viewerTwo.setCameraClipDistances(5.0,20.0);
    viewerTwo.setCameraPosition(0,0,-10, 0,1,0, 0);

    // Display the visualiser until 'q' key is pressed
    while (!viewerTwo.wasStopped ()) {
      viewerTwo.spinOnce ();
    }
    viewerTwo.close();
    ///////////////////////////////////////////////////////////////////////////////////
#endif

    ////////////////////// Backface Culling & Frustum Culling  ////////////////////////
   /* BACKFACE CULLING
    * This method removes all the faces that aren't visible to the camera.
    *
    * For a vertex to be visible it should satisfy the below equation:
    * (Vo - P).(N) < 0
    * Vo is the XYZ coorindate of the camera location
    * P is the XYZ coordiante of the vertex being tested
    * N is the XYZ coordinate of the normal of vertex P
    * . is the dot product
    *
    * This simplifies to (P).(N) < 0 as our camera is located at (0,0,0) and P represents the position vector of each vertex.
    *
    * For more info see: https://en.wikipedia.org/wiki/Back-face_culling
    *
    * ---------
    *
    * FRUSTUM CULLING
    * If the vertex Z value is less than the top of the frustum at that X location.
    * If the vertex Z value is greater than the bottom of the frustum as that X location.
    * Frustum Z value calculated from vertex X coordinate *  FrustumVectorZ/FrustumVectorX
    *
    * If the vertex Y value is less than the left of the frustum at that X location.
    * If the vertex Y value is greater than the right of the frustum at that X location.
    * Frustum Y value calculated from vertex X coordinate * FrustumVectorY/FrustumVectorX
    */

    std::vector<pcl::Vertices, std::allocator<pcl::Vertices> >::iterator face;
    pcl::PointCloud<pcl::PointXYZ> visiblePoints;
    std::vector<pcl::Vertices> visibleFaces;
    for (face = objectMesh.polygons.begin(); face != objectMesh.polygons.end(); face++)
    {
        bool isVisible = true;

        // Find the veritices of each face.
        unsigned int v1 = face->vertices[0];
        unsigned int v2 = face->vertices[1];
        unsigned int v3 = face->vertices[2];

        // Get XYZ point for each of the vertices.
        // As our camera origin is (0,0,0) at the WCS origin,
        // each point represents a vector from the camera origin to each vertex in object_cloud.
        pcl::PointXYZ p1 = objectPointCloud->points.at(v1);
        pcl::PointXYZ p2 = objectPointCloud->points.at(v2);
        pcl::PointXYZ p3 = objectPointCloud->points.at(v3);

        // Get the normal for each of the vertices.
        pcl::PointNormal n1 = objectNormalCloud->points.at(v1);
        pcl::PointNormal n2 = objectNormalCloud->points.at(v2);
        pcl::PointNormal n3 = objectNormalCloud->points.at(v3);

        // Compute the dot product for each of the vertices.
        // If the dot product is greater than or equal to 0, set isVisible to false.
        isVisible = isVisible && ((p1.x * n1.x + p1.y * n1.y + p1.z * n1.z) < 0);
        isVisible = isVisible && ((p2.x * n2.x + p2.y * n2.y + p2.z * n2.z) < 0);
        isVisible = isVisible && ((p3.x * n3.x + p3.y * n3.y + p3.z * n3.z) < 0);

        // FRUSTUM CULLING - Extra function to check that the vertex is contained within the viewing frustum.
        // Each point needs to be checked for above and below Z and to the right and left of Y. Hence 6 lines.
        isVisible = isVisible && (p1.y < p1.z*(frustumTopLeft.y/frustumTopLeft.z)) && (p1.y > p1.z*(frustumBottomLeft.y/frustumBottomLeft.z));
        isVisible = isVisible && (p1.x < p1.z*(frustumTopLeft.x/frustumTopLeft.z)) && (p1.x > p1.z*(frustumTopRight.x/frustumTopRight.z));
        isVisible = isVisible && (p2.y < p2.z*(frustumTopLeft.y/frustumTopLeft.z)) && (p2.y > p2.z*(frustumBottomLeft.y/frustumBottomLeft.z));
        isVisible = isVisible && (p2.x < p2.z*(frustumTopLeft.x/frustumTopLeft.z)) && (p2.x > p2.z*(frustumTopRight.x/frustumTopRight.z));
        isVisible = isVisible && (p3.y < p3.z*(frustumTopLeft.y/frustumTopLeft.z)) && (p3.y > p3.z*(frustumBottomLeft.y/frustumBottomLeft.z));
        isVisible = isVisible && (p3.x < p3.z*(frustumTopLeft.x/frustumTopLeft.z)) && (p3.x > p3.z*(frustumTopRight.x/frustumTopRight.z));

        // If isVisible is True, collect the points in visiblePoints.
        // If isVisible is True, collect the face vertices in visibleFaces.
        if(isVisible)
        {
            visiblePoints.push_back(p1);
            visiblePoints.push_back(p2);
            visiblePoints.push_back(p3);
            visibleFaces.push_back(*face);
        }
    }

    // Remove all faces from object_mesh
    objectMesh.polygons.clear();

    // Re-insert all faces back into object_mesh
    objectMesh.polygons.insert(objectMesh.polygons.begin(),visibleFaces.begin(),visibleFaces.end());

    // Remove the vertices that are un-used from object_mesh.
    pcl::PolygonMesh visible(objectMesh);
    pcl::surface::SimplificationRemoveUnusedVertices cleaner;
    cleaner.simplify(visible,objectMesh);

    pcl::io::savePLYFile("./Models/objectMeshCulled.ply",objectMesh);
    ///////////////////////////////////////////////////////////////////////////////////
#ifdef VIRTUAL_VIEW_BACKFACE_AND_FRUSTUM_CULLING
    // Create the visualizer
    pcl::visualization::PCLVisualizer viewerThree ("Visualize the actual scene");
    // Add objectPointCLoudWorkspaceView and cameraCloudOne
    viewerThree.addPolygonMesh(objectMesh, "objectMesh");

    // Add the point cloud to the viewer and pass the color handler
    viewerThree.addPointCloud(cameraCloudOne, cameraColorHandlerVirtualView, "VirtualCameraOne");

    // Set size of the origin point cloud
    viewerThree.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "VirtualCameraOne");

    /// Add pixelGridCloud Points ///
    //Represent pixelGridCloud
    //viewerThree.addPointCloud(pixelGridCloud, pixelGridColorHandler, "pixelGridCloud");
    viewerThree.addLine(frustumBottomRight,frustumTopRight,"rightSide",0);
    viewerThree.addLine(frustumTopRight,frustumTopLeft,"topSide",0);
    viewerThree.addLine(frustumTopLeft,frustumBottomLeft,"leftSide",0);
    viewerThree.addLine(frustumBottomLeft,frustumBottomRight,"bottomSide",0);

    //Draw frustum itself.
    viewerThree.addLine(pcl::PointXYZ(0,0,0),frustumTopRight,"topRight",0);
    viewerThree.addLine(pcl::PointXYZ(0,0,0),frustumTopLeft,"topLeft",0);
    viewerThree.addLine(pcl::PointXYZ(0,0,0),frustumBottomLeft,"bottomLeft",0);
    viewerThree.addLine(pcl::PointXYZ(0,0,0),frustumBottomRight,"bottomRight",0);

    // Add label to origin for x,y,z point position.
    viewerThree.addText3D("+X",xTextVirtualView,0.1,1.0,1.0,1.0,"x",0);
    viewerThree.addLine(pcl::PointXYZ(0,0,0),pcl::PointXYZ(2,0,0),"line_x",0);

    viewerThree.addText3D("+Y",yTextVirtualView,0.1,1.0,1.0,1.0,"y",0);
    viewerThree.addLine(pcl::PointXYZ(0,0,0),pcl::PointXYZ(0,2,0),"line_y",0);

    viewerThree.addText3D("+Z",zTextVirtualView,0.1,1.0,1.0,1.0,"z",0);
    viewerThree.addLine(pcl::PointXYZ(0,0,0),pcl::PointXYZ(0,0,2),"line_z",0);

    viewerThree.setCameraClipDistances(5.0,20.0);
    viewerThree.setCameraPosition(0,0,-10, 0,1,0, 0);

    // Display the visualiser until 'q' key is pressed
    while (!viewerThree.wasStopped ()) {
      viewerThree.spinOnce ();
    }
    viewerThree.close();
#endif

    //////////////////////// Adding Points to the surfaces  ///////////////////////////
    pcl::fromPCLPointCloud2(objectMesh.cloud,*objectPointCloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr selectedPoints (new (pcl::PointCloud<pcl::PointXYZ>));
    pcl::PointCloud<pcl::PointXYZ>::Ptr totalIntersectedPoints (new (pcl::PointCloud<pcl::PointXYZ>));
    pcl::PointCloud<pcl::PointXYZ>::Ptr selectedNoisyPoints (new (pcl::PointCloud<pcl::PointXYZ>));
    pcl::PointCloud<pcl::PointXYZ>::Ptr intersectedPixelGridLocations (new (pcl::PointCloud<pcl::PointXYZ>));
    pcl::PointCloud<pcl::PointXYZ>::iterator pixelGrid_Iterator;  // Each point in the pixelgrid of the camera
    pcl::PointXYZ p; // The point of intersection of a plane and the line.
    pcl::PointXYZ pointPixelGrid;
    float alpha;
    float beta;
    float gamma;
    std::vector<float> phiValues;
    float totalFaceArea = 0;

    // The point at which the line from the camera focalpoint intersects the face of object_mesh
    for (face = objectMesh.polygons.begin(); face != objectMesh.polygons.end(); face++)
    {

        pcl::PointCloud<pcl::PointXYZ>::Ptr intersectedPoints (new (pcl::PointCloud<pcl::PointXYZ>));

        // Find the vertices of each face.
        unsigned int v1 = face->vertices[0];
        unsigned int v2 = face->vertices[1];
        unsigned int v3 = face->vertices[2];

        // Get XYZ point for each of the vertices.
        // As our camera origin is (0,0,0) at the WCS origin,
        // each point represents a vector from the camera origin to each vertex in object_cloud.
        pcl::PointXYZ p1 = objectPointCloud->points.at(v1);
        pcl::PointXYZ p2 = objectPointCloud->points.at(v2);
        pcl::PointXYZ p3 = objectPointCloud->points.at(v3);

        // Find equation of the plane from 3 points. p1,p2,p3
        // More info: http://www.had2know.com/academics/equation-plane-through-3-points.html
        // Plane vector 1
        pcl::PointXYZ u1;
        u1.x = p2.x - p1.x;
        u1.y = p2.y - p1.y;
        u1.z = p2.z - p1.z;

        // Plane vector 2
        pcl::PointXYZ u2;
        u2.x = p3.x - p1.x;
        u2.y = p3.y - p1.y;
        u2.z = p3.z - p1.z;

        // Compute the cross product between the two vectors. This calculate the equation of the plane. u3[0] = a, u3[1] = b, u3[2] = c, u3[3] = d
        // Also the normal to the plane.
        pcl::PointXYZ u3; //pcl::PointXYZ not pcl::PointNormal so the same function can be used twice.
        u3.x = u1.y*u2.z - u1.z*u2.y;  // plane component a
        u3.y = u1.z*u2.x - u1.x*u2.z;  // plane component b
        u3.z = u1.x*u2.y - u1.y*u2.x;  // plane component c

        // Calculate face surface area.
        float u1u2angle = dotProductAngle(u1, u2);
        float u1Magnitude = sqrt(pow(u1.x,2) + pow(u1.y,2) + pow(u1.z,2));
        float u2Magnitude = sqrt(pow(u2.x,2) + pow(u2.y,2) + pow(u2.z,2));
        float faceArea = 0.5*(u1Magnitude*u2Magnitude)*sin(u1u2angle);

        totalFaceArea = totalFaceArea + faceArea;

        int pointIntersectionCounter = 0;

        phiValues.clear(); // Ensure empty vector varaible before counting the ray phi angles.

        // Work through all rays, to find which intersect and are contained within the area of the 3 coordinates.
        for(pixelGrid_Iterator = pixelGridCloud->points.begin(); pixelGrid_Iterator != pixelGridCloud->end(); pixelGrid_Iterator++)
        {
            bool isIntersected = true;

            //Normalise the directional vectors of the pixel grid.
            pointPixelGrid.x = (pixelGrid_Iterator->x) ;// sqrt(pow(pixelGrid_Iterator->x,2)+pow(pixelGrid_Iterator->y,2)+pow(pixelGrid_Iterator->z,2));
            pointPixelGrid.y = (pixelGrid_Iterator->y) ;// sqrt(pow(pixelGrid_Iterator->x,2)+pow(pixelGrid_Iterator->y,2)+pow(pixelGrid_Iterator->z,2));
            pointPixelGrid.z = (pixelGrid_Iterator->z) ;// sqrt(pow(pixelGrid_Iterator->x,2)+pow(pixelGrid_Iterator->y,2)+pow(pixelGrid_Iterator->z,2));

            float d_top = p1.x * u3.x + p1.y * u3.y + p1.z * u3.z;
            // If d_top is 0 then then every point on the line intersect the plane, i.e. the line fits in the plane.
            float d_bottom = pointPixelGrid.x*u3.x + pointPixelGrid.y*u3.y + pointPixelGrid.z*u3.z;
            // If d_bottom is 0 then the line is parallel to the plane, this may not occur as backface culling should already have removed these occurances.
            float d = d_top / d_bottom;

            // Check to make sure the lines aren't parallel, only case when they won't intersect.
            isIntersected = isIntersected && (d != 0);

            // Calculate points of intersection
            p.x = (pointPixelGrid.x)*d;
            p.y = (pointPixelGrid.y)*d;
            p.z = (pointPixelGrid.z)*d;

            // Calculate if intersected point lies within the triangle of the 3 vertices.
            // More Info: http://stackoverflow.com/questions/13300904/determine-whether-point-lies-inside-triangle
            alpha = ((p2.y - p3.y)*(p.x - p3.x) + (p3.x - p2.x)*(p.y - p3.y)) / ((p2.y - p3.y)*(p1.x - p3.x) + (p3.x - p2.x)*(p1.y - p3.y));
            beta = ((p3.y - p1.y)*(p.x - p3.x) + (p1.x - p3.x)*(p.y - p3.y)) / ((p2.y - p3.y)*(p1.x - p3.x) + (p3.x - p2.x)*(p1.y - p3.y));
            gamma = 1.0f - alpha - beta;


            isIntersected = isIntersected && (alpha > 0) && (beta > 0) && (gamma > 0);

            // If isIntersected is True, collect the points in instersectedPoints.
            if(isIntersected)
            {
                intersectedPixelGridLocations->push_back(pointPixelGrid);
                totalIntersectedPoints->push_back(p);
                intersectedPoints->push_back(p);
                phiValues.push_back(180-(dotProductAngle(pointPixelGrid,u3)*180/M_PI));
                if((180-(dotProductAngle(pointPixelGrid,u3)*180/M_PI)) >= 90)
                {
                    cout << "angle greater than 90" << endl;
                    break;
                }
//                cout << pointPixelGrid.x << " " << pointPixelGrid.y << " " <<pointPixelGrid.z << endl;
//                cout << u3.x << " " << u3.y << " " << u3.z << endl;
                pointIntersectionCounter++;

            }
        }
        // Choose which points to keep or not.
        float perfectPointDensity = pointIntersectionCounter/(faceArea*1000000);

        for(int iCounter = 0; iCounter < intersectedPoints->points.size(); iCounter++)
        {
            bool isUnderMaxAngle = true;

            isUnderMaxAngle = isUnderMaxAngle && (phiValues[iCounter] <= 55);

            if(isUnderMaxAngle)
            {
                bool isSuccessfull = true;
                float rProjected = (53/2)*cos(phiValues[iCounter]*M_PI/180);
                float areaProjected = pow(rProjected,2)*M_PI;
                float realTotalPointCount = pointCountSample(phiValues[iCounter],sampleSelector);
                float realPointDensity = realTotalPointCount / areaProjected;
                float pointProbability = realPointDensity / perfectPointDensity;

                float randomNumberGenerator = ((double) rand() / (RAND_MAX)); // Random Number Generator between 0 and 1.

                isSuccessfull = isSuccessfull && (randomNumberGenerator <= pointProbability);

                if(isSuccessfull)
                {
                    selectedPoints->push_back(intersectedPoints->points[iCounter]);

                    float intersectedPixelGridLocationsMagnitude = sqrt(pow(intersectedPixelGridLocations->points[iCounter].x,2)+
                                                                        pow(intersectedPixelGridLocations->points[iCounter].y,2)+
                                                                        pow(intersectedPixelGridLocations->points[iCounter].z,2));

                    pcl::PointXYZ iPGLUnit;
                    iPGLUnit.x = intersectedPixelGridLocations->points[iCounter].x/intersectedPixelGridLocationsMagnitude;
                    iPGLUnit.y = intersectedPixelGridLocations->points[iCounter].y/intersectedPixelGridLocationsMagnitude;
                    iPGLUnit.z = intersectedPixelGridLocations->points[iCounter].z/intersectedPixelGridLocationsMagnitude;

                    pcl::PointXYZ noisyPoint;
                    noisyPoint.x = intersectedPoints->points[iCounter].x + iPGLUnit.x*norm_rnd();
                    noisyPoint.y = intersectedPoints->points[iCounter].y + iPGLUnit.y*norm_rnd();
                    noisyPoint.z = intersectedPoints->points[iCounter].z + iPGLUnit.z*norm_rnd();

                    selectedNoisyPoints->push_back(noisyPoint);
                }
            }
        }
    }
    pcl::io::savePCDFileASCII("./Models/totalIntersectedPointCloud.pcd",*totalIntersectedPoints);
    pcl::io::savePCDFileASCII("./Models/selectedPointCloud.pcd",*selectedPoints);
    pcl::io::savePCDFileASCII("./Models/selectedNoisyPointCloud.pcd",*selectedNoisyPoints);

    cout << "Average points/mm^2 = " << selectedPoints->points.size() / (totalFaceArea*1000000) << "p/mm^2" << endl;

    ///////////////////////////////////////////////////////////////////////////////////
#ifdef VIRTUAL_VIEW_INTERSECTED_POINTS

    // Create the visualizer
    pcl::visualization::PCLVisualizer viewerFour ("Visualize the simulated scene");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> totalIntersectedPointsColorHandler (totalIntersectedPoints, 255, 0, 255);

    viewerFour.addPointCloud(totalIntersectedPoints, totalIntersectedPointsColorHandler, "totalIntersectedPoints");

    // Add the point cloud to the viewer and pass the color handler
    viewerFour.addPointCloud(cameraCloudOne, cameraColorHandlerVirtualView, "VirtualCameraOne");

    // Set size of the origin point cloud
    viewerFour.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "VirtualCameraOne");

    /// Add pixelGridCloud Points ///
    //Represent pixelGridCloud
    //viewerFour.addPointCloud(pixelGridCloud, pixelGridColorHandler, "pixelGridCloud");
    viewerFour.addLine(frustumBottomRight,frustumTopRight,"rightSide",0);
    viewerFour.addLine(frustumTopRight,frustumTopLeft,"topSide",0);
    viewerFour.addLine(frustumTopLeft,frustumBottomLeft,"leftSide",0);
    viewerFour.addLine(frustumBottomLeft,frustumBottomRight,"bottomSide",0);

    //Draw frustum itself.
    viewerFour.addLine(pcl::PointXYZ(0,0,0),frustumTopRight,"topRight",0);
    viewerFour.addLine(pcl::PointXYZ(0,0,0),frustumTopLeft,"topLeft",0);
    viewerFour.addLine(pcl::PointXYZ(0,0,0),frustumBottomLeft,"bottomLeft",0);
    viewerFour.addLine(pcl::PointXYZ(0,0,0),frustumBottomRight,"bottomRight",0);

    // Add label to origin for x,y,z point position.
    viewerFour.addText3D("+X",xTextVirtualView,0.1,1.0,1.0,1.0,"x",0);
    viewerFour.addLine(pcl::PointXYZ(0,0,0),pcl::PointXYZ(2,0,0),"line_x",0);

    viewerFour.addText3D("+Y",yTextVirtualView,0.1,1.0,1.0,1.0,"y",0);
    viewerFour.addLine(pcl::PointXYZ(0,0,0),pcl::PointXYZ(0,2,0),"line_y",0);

    viewerFour.addText3D("+Z",zTextVirtualView,0.1,1.0,1.0,1.0,"z",0);
    viewerFour.addLine(pcl::PointXYZ(0,0,0),pcl::PointXYZ(0,0,2),"line_z",0);

    viewerFour.setCameraClipDistances(5.0,20.0);
    viewerFour.setCameraPosition(0,0,-10, 0,1,0, 0);

    // Display the visualiser until 'q' key is pressed
    while (!viewerFour.wasStopped ()) {
      viewerFour.spinOnce ();
    }
    viewerFour.close();
#endif

#ifdef VIRTUAL_VIEW_SELECTED_POINTS
    // View just the selected points
    // Create the visualizer
    pcl::visualization::PCLVisualizer viewerFive ("Visualize the simulated scene");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> selectedPointsColorHandler (selectedPoints, 255, 0, 255);

    viewerFive.addPointCloud(selectedPoints, selectedPointsColorHandler, "selectedPoints");

    // Add the point cloud to the viewer and pass the color handler
    viewerFive.addPointCloud(cameraCloudOne, cameraColorHandlerVirtualView, "VirtualCameraOne");

    // Set size of the origin point cloud
    viewerFive.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "VirtualCameraOne");

    /// Add pixelGridCloud Points ///
    //Represent pixelGridCloud
    //viewerFive.addPointCloud(pixelGridCloud, pixelGridColorHandler, "pixelGridCloud");
    viewerFive.addLine(frustumBottomRight,frustumTopRight,"rightSide",0);
    viewerFive.addLine(frustumTopRight,frustumTopLeft,"topSide",0);
    viewerFive.addLine(frustumTopLeft,frustumBottomLeft,"leftSide",0);
    viewerFive.addLine(frustumBottomLeft,frustumBottomRight,"bottomSide",0);

    //Draw frustum itself.
    viewerFive.addLine(pcl::PointXYZ(0,0,0),frustumTopRight,"topRight",0);
    viewerFive.addLine(pcl::PointXYZ(0,0,0),frustumTopLeft,"topLeft",0);
    viewerFive.addLine(pcl::PointXYZ(0,0,0),frustumBottomLeft,"bottomLeft",0);
    viewerFive.addLine(pcl::PointXYZ(0,0,0),frustumBottomRight,"bottomRight",0);

    // Add label to origin for x,y,z point position.
    viewerFive.addText3D("+X",xTextVirtualView,0.1,1.0,1.0,1.0,"x",0);
    viewerFive.addLine(pcl::PointXYZ(0,0,0),pcl::PointXYZ(2,0,0),"line_x",0);

    viewerFive.addText3D("+Y",yTextVirtualView,0.1,1.0,1.0,1.0,"y",0);
    viewerFive.addLine(pcl::PointXYZ(0,0,0),pcl::PointXYZ(0,2,0),"line_y",0);

    viewerFive.addText3D("+Z",zTextVirtualView,0.1,1.0,1.0,1.0,"z",0);
    viewerFive.addLine(pcl::PointXYZ(0,0,0),pcl::PointXYZ(0,0,2),"line_z",0);

    viewerFive.setCameraClipDistances(5.0,20.0);
    viewerFive.setCameraPosition(0,0,-10, 0,1,0, 0);

    // Display the visualiser until 'q' key is pressed
    while (!viewerFive.wasStopped ()) {
      viewerFive.spinOnce ();
    }
    viewerFive.close();
#endif

#ifdef VIRTUAL_VIEW_SELECTED_NOISY_POINTS
    // View just the selected points
    // Create the visualizer
    pcl::visualization::PCLVisualizer viewerSix ("Visualize the simulated scene with noise");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> selectedNoisyPointsColorHandler (selectedNoisyPoints, 255, 0, 255);

    viewerSix.addPointCloud(selectedNoisyPoints, selectedNoisyPointsColorHandler, "selectedNoisyPoints");

    // Add the point cloud to the viewer and pass the color handler
    viewerSix.addPointCloud(cameraCloudOne, cameraColorHandlerVirtualView, "VirtualCameraOne");

    // Set size of the origin point cloud
    viewerSix.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "VirtualCameraOne");

    /// Add pixelGridCloud Points ///
    //Represent pixelGridCloud
    //viewerSix.addPointCloud(pixelGridCloud, pixelGridColorHandler, "pixelGridCloud");
    viewerSix.addLine(frustumBottomRight,frustumTopRight,"rightSide",0);
    viewerSix.addLine(frustumTopRight,frustumTopLeft,"topSide",0);
    viewerSix.addLine(frustumTopLeft,frustumBottomLeft,"leftSide",0);
    viewerSix.addLine(frustumBottomLeft,frustumBottomRight,"bottomSide",0);

    //Draw frustum itself.
    viewerSix.addLine(pcl::PointXYZ(0,0,0),frustumTopRight,"topRight",0);
    viewerSix.addLine(pcl::PointXYZ(0,0,0),frustumTopLeft,"topLeft",0);
    viewerSix.addLine(pcl::PointXYZ(0,0,0),frustumBottomLeft,"bottomLeft",0);
    viewerSix.addLine(pcl::PointXYZ(0,0,0),frustumBottomRight,"bottomRight",0);

    // Add label to origin for x,y,z point position.
    viewerSix.addText3D("+X",xTextVirtualView,0.1,1.0,1.0,1.0,"x",0);
    viewerSix.addLine(pcl::PointXYZ(0,0,0),pcl::PointXYZ(2,0,0),"line_x",0);

    viewerSix.addText3D("+Y",yTextVirtualView,0.1,1.0,1.0,1.0,"y",0);
    viewerSix.addLine(pcl::PointXYZ(0,0,0),pcl::PointXYZ(0,2,0),"line_y",0);

    viewerSix.addText3D("+Z",zTextVirtualView,0.1,1.0,1.0,1.0,"z",0);
    viewerSix.addLine(pcl::PointXYZ(0,0,0),pcl::PointXYZ(0,0,2),"line_z",0);

    viewerSix.setCameraClipDistances(5.0,20.0);
    viewerSix.setCameraPosition(0,0,-10, 0,1,0, 0);

    tend = time(0);
    cout << "It took " << difftime(tend,tstart) << " second(s)." << endl;

    // Display the visualiser until 'q' key is pressed
    while (!viewerSix.wasStopped ()) {
      viewerSix.spinOnce ();
    }
    viewerSix.close();
#endif


}
