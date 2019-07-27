#include <iostream>
#include <string>
#include <pcl/io/ply_io.h>  
#include <pcl/point_types.h>  
#include <pcl/registration/icp.h>  
#include <pcl/visualization/pcl_visualizer.h>  
#include <pcl/console/time.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

bool spaceKeyPressed = false;

void print4x4Matrix(const Eigen::Matrix4d & matrix)
{
	printf("Rotation matrix :\n");
	printf("    | %6.3f %6.3f %6.3f | \n", matrix(0, 0), matrix(0, 1), matrix(0, 2));
	printf("R = | %6.3f %6.3f %6.3f | \n", matrix(1, 0), matrix(1, 1), matrix(1, 2));
	printf("    | %6.3f %6.3f %6.3f | \n", matrix(2, 0), matrix(2, 1), matrix(2, 2));
	printf("Translation vector :\n");
	printf("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix(0, 3), matrix(1, 3), matrix(2, 3));
}

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* nothing)
{
	if (event.getKeySym() == "space" && event.keyDown())
		spaceKeyPressed = true;
}

int main()
{
	PointCloudT::Ptr cloud_in(new PointCloudT);  // Original point cloud  
	PointCloudT::Ptr cloud_tr(new PointCloudT);  // Transformed point cloud  
	PointCloudT::Ptr cloud_icp(new PointCloudT);  // ICP output point cloud  

	int iterationCount = 0; 
	pcl::console::TicToc time;
	time.tic();
	std::string filename = "test.ply";
	if (pcl::io::loadPLYFile(filename, *cloud_in) < 0)
	{
		PCL_ERROR("Error loading cloud %s.\n", filename);
		system("pause");
		return (-1);
	}

	float rat = 0.001f;//convert to meter(m)
	for (size_t i = 0; i < cloud_in->points.size(); ++i)
	{
		cloud_in->points[i].x *= rat;
		cloud_in->points[i].y *= rat;
		cloud_in->points[i].z *= rat;
	}

	std::cout << "\nLoaded file " 
		<< filename << " (" << cloud_in->size() << " points) in " 
		<< time.toc() << " ms\n" 
		<< std::endl;

	Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();
	double theta = M_PI / 4; 
	transformation_matrix(0, 0) = cos(theta);
	transformation_matrix(0, 1) = -sin(theta);
	transformation_matrix(1, 0) = sin(theta);
	transformation_matrix(1, 1) = cos(theta);
	transformation_matrix(2, 3) = 0.4;

	std::cout << "Applying this rigid transformation to: cloud_in -> cloud_icp" << std::endl;
	print4x4Matrix(transformation_matrix);

	pcl::transformPointCloud(*cloud_in, *cloud_icp, transformation_matrix);
	*cloud_tr = *cloud_icp;

	time.tic();

	pcl::IterativeClosestPoint<PointT, PointT> icp;
	icp.setInputSource(cloud_icp);
	icp.setInputTarget(cloud_in);
	icp.setMaximumIterations(iterationCount);

	pcl::visualization::PCLVisualizer viewer("ICP demo");
	int v1(0);
	int v2(1);
	viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);

	float bckgr_color = 0.0;
	float txt_color = 1.0 - bckgr_color;

	//Original white point cloud.
	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h(cloud_in, (int)255 * txt_color, (int)255 * txt_color,
		(int)255 * txt_color);
	viewer.addPointCloud(cloud_in, cloud_in_color_h, "cloud_in_v1", v1);
	viewer.addPointCloud(cloud_in, cloud_in_color_h, "cloud_in_v2", v2);

	//Transformed point cloud is green.
	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_tr_color_h(cloud_tr, 180, 20, 20);
	viewer.addPointCloud(cloud_tr, cloud_tr_color_h, "cloud_tr_v1", v1);

	//ICP aligned point cloud is red.
	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_icp_color_h(cloud_icp, 180, 20, 20);
	viewer.addPointCloud(cloud_icp, cloud_icp_color_h, "cloud_icp_v2", v2);

	//Adding text descriptions in each viewport.
	viewer.addText("White: Original point cloud\nGreen: Matrix transformed point cloud", 10, 15, 16, txt_color, txt_color, txt_color, "icp_info_1", v1);
	viewer.addText("White: Original point cloud\nRed: ICP aligned point cloud", 10, 15, 16, txt_color, txt_color, txt_color, "icp_info_2", v2);

	std::stringstream ss;
	ss << iterationCount;
	std::string iterations_cnt = "ICP iterations = " + ss.str();
	viewer.addText(iterations_cnt, 10, 60, 16, txt_color, txt_color, txt_color, "iterations_cnt", v2);
	viewer.setBackgroundColor(bckgr_color, bckgr_color, bckgr_color, v1);
	viewer.setBackgroundColor(bckgr_color, bckgr_color, bckgr_color, v2);
	viewer.setCameraPosition(-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
	viewer.setSize(960, 540);
    viewer.registerKeyboardCallback(&keyboardEventOccurred, (void*)NULL);

	while (!viewer.wasStopped())
	{
		viewer.spinOnce();

		if (spaceKeyPressed)
		{
			time.tic();

			icp.align(*cloud_icp);

			std::cout << "Applied " << iterationCount 
				<< " ICP iteration(s) in " << time.toc() << " ms" 
				<< std::endl;

			if (icp.hasConverged())
			{
				std::cout << "ICP has converged, score is %+.0e\n" << icp.getFitnessScore()
					<< "\nICP transformation " << ++iterationCount 
					<< " : cloud_icp -> cloud_in" << std::endl;

				transformation_matrix *= icp.getFinalTransformation().cast<double>();
				print4x4Matrix(transformation_matrix);//Print matrix between original pose and current pose

				ss.str("");
				ss << iterationCount;
				std::string iterations_cnt = "ICP iterations = " + ss.str();
				viewer.updateText(iterations_cnt, 10, 60, 16, txt_color, txt_color, txt_color, "iterations_cnt");
				viewer.updatePointCloud(cloud_icp, cloud_icp_color_h, "cloud_icp_v2");
			}
			else
			{
				PCL_ERROR("\nICP has not converged.\n");
				system("pause");
				return (-1);
			}
		}
		spaceKeyPressed = false;
	}
	system("pause");
	return (0);
}