#include <iostream>
#include </usr/include/boost/thread/thread.hpp>

#include <pcl/common/common_headers.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_picking_event.h>
//#include <pcl/console/parse.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>

using namespace std;

int i;
int PointInd;
Eigen::VectorXi indic;

void ExRaNeighbor(int qpointInd,pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_in,float radius, Eigen::VectorXi label)
//this function gets indexes of a points neighbors.
{
	pcl::KdTreeFLANN<pcl::PointXYZRGBL> tree;
	vector<int>   IdRadiusSearch;
	vector<float> Sdistance;

	tree.setInputCloud(cloud_in);
    pcl::PointXYZRGBL qpoint = cloud_in->points[qpointInd];
	if(tree.radiusSearch(qpoint,radius,IdRadiusSearch,Sdistance) > 0)
	{
		cout<<"Points count= "<<IdRadiusSearch.size()<<endl;
	}
	else
		cerr<<"Not enough points in the neighborhood!"<<endl;
	label[qpointInd] = 1;
	cloud_in->points[qpointInd].label = 1;
	for (size_t k = 0; k < IdRadiusSearch.size(); k++)
	{
		label[IdRadiusSearch[k]] = 1;
		cloud_in->points[IdRadiusSearch[k]].label = 1;
	}

}
//--------------------------------------------------------------------------------
//--------------------------------------------------------------------------------

void pp_callback (const pcl::visualization::PointPickingEvent& event, void* viewer_void)
{
 if(i < indic.rows())
 {
  PointInd = event.getPointIndex();
  if (PointInd == -1)
  {
    return;
  }
  indic[i] = PointInd;
 }
 else
	 cout<<"Number of points is exceeded size of vector!"<< endl;
  cout <<"Point number "<< i <<": "<< PointInd << endl;
  i = i+1;

}
//----------------------------------------------------------------------------------
boost::shared_ptr<pcl::visualization::PCLVisualizer> interactionCustomizationVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  //viewer->addCoordinateSystem (1.0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb,"sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer->registerPointPickingCallback (pp_callback, (void*)&viewer);
  return (viewer);
}

// ----------------------------------------------------------------------------------
// -----Main-----
// --------------
int
main (int argc, char** argv)
//First input arg is the path of the input cloud
//Second would be the path of the labeled pointcloud
//Third would be a switch (1: Annotation 2: seeing result)
//The rest args would be radius values for the objects being annotated.
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr basic_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGBL>::Ptr test (new pcl::PointCloud<pcl::PointXYZRGBL>);
  //pcl::visualization::PCLVisualizer viewer2("tamasha");
  Eigen::VectorXi LabeledInd;



  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (argv[1], *basic_cloud_ptr) == -1) //* load the file
   {
     PCL_ERROR ("Couldn't read file \n");
     return (-1);
   }


  if(atoi(argv[3]) == 1)
  {
  pcl::copyPointCloud(*basic_cloud_ptr,*test);
  indic.setZero(argc - 4);//initializes indic based on the number of radiuses entered as input.

  std::cout<<"indices:"<<indic.rows()<<std::endl;
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  viewer = interactionCustomizationVis(basic_cloud_ptr);

  //--------------------
  // -----Main loop-----
  //--------------------

  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
  cout << "indixes :" << indic << endl;
  cout << "argc = " << argc;
  //----------------------------------
  float r;
  LabeledInd.setZero(basic_cloud_ptr->size());

  for (int j = 0; j < indic.rows(); j++)
  {
	  r = (atof(argv[j+4]))/100;
	  //ExRaNeighbor(indic[j], basic_cloud_ptr, r, LabeledInd);
	  ExRaNeighbor(indic[j], test, r, LabeledInd);
  }

cout<<"the result has "<< LabeledInd.rows() <<" rows"<<endl;

//witing the vector to a file
ofstream out;
out.open("AnnotationTest.txt", ios::out | ios::trunc | ios::binary);
out << LabeledInd <<endl;
out.close();

pcl::io::savePCDFileASCII (argv[2], *test);
//pcl::io::savePCDFileBinary("test.pcd",basic_cloud_ptr);

  }
  //----------------------------------------------------
  //visualizing previously labeled data
  else
	  if(atoi(argv[3]) == 2)
	  {
		  if (pcl::io::loadPCDFile<pcl::PointXYZRGBL> (argv[2], *test) == -1) //* load the file
		     {
		       PCL_ERROR ("Couldn't read file \n");
		       return (-1);
		     }
		  pcl::visualization::PCLVisualizer viewer2 ("Results");

		  for( size_t h; h < basic_cloud_ptr->size(); h++)
		  {
			  if (test->points[h].label == 1)
			  basic_cloud_ptr->points[h].r = 255;
		  }
		  viewer2.addPointCloud(basic_cloud_ptr);
		  while (!viewer2.wasStopped ())
		      {
		        viewer2.spinOnce ();
		      }
	  }
	  else
		  cerr<<"Not enough CORRECT ARGUMENTS!!"<<endl;
  /*viewer2.addPointCloud(basic_cloud_ptr);
  //viewer2.addSphere(basic_cloud_ptr->points[indic[0]], 0.1, 0.5, 0.5, 0,"sphere");
  viewer2.addSphere(basic_cloud_ptr->points[169281], 0.1, 0.5, 0.5, 0,"sphere");
  while (!viewer2.wasStopped ())
    {
      viewer->spinOnce ();
    }
  for (int j = 0; j < indic.rows(); j++)
	  if(indic[j])
	    std::cout << "indice "<< j << std::endl;
*/
}
