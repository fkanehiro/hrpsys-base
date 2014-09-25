#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <vtkRenderWindow.h>
#include <vtkRendererCollection.h>
#include <vtkCamera.h>

std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds;
int rank=0;

void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                            void* viewer_void)
{
  pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);
  if (event.keyDown()){
    if (event.getKeySym () == "n" && rank+1 < clouds.size()){
      rank++;
      viewer->removePointCloud("test");
      viewer->addPointCloud(clouds[rank], "test");
    }else if (event.getKeySym() == "p" && rank-1 >= 0){
      rank--;
      viewer->removePointCloud("test");
      viewer->addPointCloud(clouds[rank], "test");
    }else if (event.getKeySym() == "r"){
      viewer->setCameraPosition(0,10,0,0,0,0,0,0,1);
    }
  }
  std::cout << rank+1 << "/" << clouds.size() << std::endl;
}


int main(int argc, char *argv[])
{
  if (argc < 2){
    std::cerr << "Usage:" << argv[0] << "[point cloud log file]" << std::endl;
    return 1;
  }

  std::ifstream ifs(argv[1]);
  if (!ifs.is_open()){
    std::cerr << "can't open " << argv[1] << std::endl;
    return 2;
  }

  double tm;
  int w, h, npoint, r=255,g=255,b=255;
  std::string type;
  float x,y,z;

  ifs >> tm;
  while (!ifs.eof()){
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    ifs >> w >> h >> type >> npoint;
    cloud->points.resize(npoint);
    for (int i=0; i<npoint; i++){
      ifs >> x >> y >> z;
      if (type == "xyzrgb"){
	ifs >> r >> g >> b;
      }
      cloud->points[i].x = x;
      cloud->points[i].y = y;
      cloud->points[i].z = z;
      cloud->points[i].r = r;
      cloud->points[i].g = g;
      cloud->points[i].b = b;
    } 
    clouds.push_back(cloud);
    ifs >> tm;
  }
  if (clouds.size()==0) return 3;

  pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
  viewer.getRenderWindow()->GetRenderers()->GetFirstRenderer()->GetActiveCamera()->SetParallelProjection(1);
  viewer.registerKeyboardCallback (keyboardEventOccurred, (void*)&viewer);
  viewer.setCameraPosition(0, 10, 0, 0, 0, 0, 0, 0, 1);
  viewer.setCameraClipDistances(1, 100);
  viewer.addPointCloud(clouds[0], "test");

  while (!viewer.wasStopped ()){
    viewer.spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));  
  }

  return 0;
}
