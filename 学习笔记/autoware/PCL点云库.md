# PCL点云库

## 一、点云格式

### 1、PCD

**PCD(Point Cloud Date，点云数据)** 对应的文件格式为**（.pcd）** ，是PCL官方指定的存储格式，具有ASCII和Binary两种数据存储类型，pcd格式具有文件头，用于描述点云的整体信息。



### 2、PCD文件格式说明

代码如下：

```
# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z
SIZE 4 4 4
TYPE F F F
COUNT 1 1 1
WIDTH 278
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS 278
DATA acsii
```

- VERSION：PCD文件版本

- FIELDS：指定一个点可以有每一个维度和字段的名字，例如

  ```
  FIELDS x y z                                   // XYZ data
  FIELDS x y z rgb                               // XYZ + colors
  FIELDS x y z normal_xnormal_y normal_z         // XYZ + surface normals
  FIELDS j1 j2 j3                                // moment invariants
  ```

- SIZE：用字节指定每个维度的大小，例如

  ```
  unsigned char/char // 1 byte
  unsigned short/short // 2 bytes
  unsigned int/int/float // 4 bytes
  double // 8 bytes
  ```

- TYPE：用一个字符指定每一维度的类型，现在被接受的类型有

  ```
  I – 表示有符号类型 int8（char）、int16（short）和int32（int）；
  U – 表示无符号类型 uint8（unsigned char）、uint16（unsigned short）和uint32（unsigned int）；
  F – 表示浮点类型。
  ```

- COUNT：指定每一个维度包含的元素数目

  ```
  默认情况下，如果没有COUNT, 所有维度的数目被设置成 1 
  ```

- WIDTH、HEIGHT：用点的数量表示点云数据集的宽度

  ```
  1)它能确定无序数据集的点云中点的个数；
  2)它能确定有序点云数据集的宽度（一行中点的数目）。
  //有序点云例子：
  WIDTH 640 //像图像一样的有序结构，有640行和480列，
  HEIGHT 480 // 这样该数据集中共有640*480=307200个点
  //无序点云：
  WIDTH 307200
  HEIGHT 1 //有307200个点的无序点云数据集
  ```

- VIEWPOINT：指定数据集中点云的获取视点

  ```
  VIEWPOINT有可能在不同坐标系之间转换的时候应用，在辅助获取其他特征时也比较有用，例如曲面法线，在判断方向一致性时，需要知道视点的方位，
  视点信息被指定为平移（tx, ty, tz）+ 四元数（qw, qx, qy, qz）。
  默认值是：VIEWPOINT 0 0 0 1 0 0 0 
  ```

- POINTS：指定点云中点的总数，例如

  ```
  POINTS 307200 //点云中点的总数为307200
  ```

- DATA：指定存储点云数据的数据类型

  ```
  从0.7版本开始，支持两种数据类型：：ASCII形式和binary形式
  ```

- **注意：**
  - **文件头DATA的下一字节就被看成是点云的数据部分，它会被解释成点云数据**
  -  **PCD文件的文件头部分必须以上面介绍的顺序精确指定**



### 3、数据存储类型（ASCII/Binary）

在0.7版本中，.PCD文件格式用两种模式存储数据：

- ASCII形式：每一点放在一个新行上

  ```
  p_1
  p_2
  p_3
  p_4
  ...
  p_n
  ```

注意： 从PCL 1.0.1版本开始，用字符串“nan”表示NaN，此字符表示该点的值不存在或非法等。

- binary格式：数据是pcl::PointCloud::points数组或者vector的完整内存复制。Linux系统中，使用mmap/munmap操作使得数据读/写尽可能快



### 4、PLY、LAS等常见点云数据格式与PCD的转换

#### LAS与PCD

LAS是美国摄影测量与遥感协会（ASPRS）所创建的和维护的行业格式。是一种用于激光雷达数据交换的已发布标准文件格式，它保留与激光雷达数据有关的特定消息。

**LibLAS**是一套用于处理常见的”LAS“LIDAR格式数据的C++函数库，这里利用LibLAS库结合PCL点云库进行LAS和PCD两种点云类型的转换。主要利用LibLAS库中`liblas::Reader`类和`liblas::Writer`类分别实现LAS数据的读取和写入。

具体地，利用`liblas::Reader`类读入LAS数据，然后利用`reader.GetPoint().GetX()`、`reader.GetPoint().GetY()`、`reader.GetPoint().GetZ()`获取LAS文件每个点的x、y、z坐标赋予PCL的cloud类存储的点的x、y、z点坐标，实现LAS数据格式向PCD数据格式的转换；将PCL的cloud类存储的点的x、y、z坐标提取并赋予`liblas::Point`类构造的点对象，然后利用`liblas::Writer`类下的WritePoint函数写入LAS类型文件，实现PCD数据格式向LAS数据格式的转换。

代码如下：

```c++
// las2pcd.cpp
#include <iostream>
#include <cstdlib>
#include <liblas/liblas.hpp>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

using namespace std;

int main (int argc, char** argv)
{
    std::ifstream ifs(argv[1], std::ios::in | std::ios::binary); // 打开las文件
    liblas::ReaderFactory f;
    liblas::Reader reader = f.CreateWithStream(ifs); // 读取las文件

	unsigned long int nbPoints=reader.GetHeader().GetPointRecordsCount();//获取las数据点的个数

	pcl::PointCloud<pcl::PointXYZRGB> cloud;
	cloud.width    = nbPoints;	//保证与las数据点的个数一致	
	cloud.height   = 1;			
	cloud.is_dense = false;
	cloud.points.resize (cloud.width * cloud.height);

	int i=0;				
	uint16_t r1, g1, b1;	
	int r2, g2, b2;			
	uint32_t rgb;			

	while(reader.ReadNextPoint()) 
	{
		// 获取las数据的x，y，z信息
		cloud.points[i].x = (reader.GetPoint().GetX());
	    cloud.points[i].y = (reader.GetPoint().GetY());
	    cloud.points[i].z = (reader.GetPoint().GetZ());
		
		//获取las数据的r，g，b信息
		r1 = (reader.GetPoint().GetColor().GetRed());
		g1 = (reader.GetPoint().GetColor().GetGreen());
		b1 = (reader.GetPoint().GetColor().GetBlue()); 
		r2 = ceil(((float)r1/65536)*(float)256);
		g2 = ceil(((float)g1/65536)*(float)256);
		b2 = ceil(((float)b1/65536)*(float)256);
		rgb = ((int)r2) << 16 | ((int)g2) << 8 | ((int)b2);
		cloud.points[i].rgb = *reinterpret_cast<float*>(&rgb);
					
		i++; 
	}
  
	pcl::io::savePCDFileASCII ("pointcloud.pcd", cloud);//存储为pcd类型文件
	return (0);
}
```



#### PLY与PCD

PLY是一种常见的点云存储格式，由斯坦福大学开发，其最早主要用于存储三维扫码仪器的点云数据。同PCD格式文件一样，PLY也有两种文件格式：ASCII和二进制。

代码如下：

```c++
// ply2pcd.cpp
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

void
printHelp (int, char **argv)
{
  print_error ("Syntax is: %s [-format 0|1] input.ply output.pcd\n", argv[0]);
}
// 读取PLY类型文件
bool loadCloud (const std::string &filename, pcl::PCLPointCloud2 &cloud)
{
  TicToc tt;
  print_highlight ("Loading "); print_value ("%s ", filename.c_str ());

  pcl::PLYReader reader;
  tt.tic ();
  if (reader.read (filename, cloud) < 0)
    return (false);
  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", cloud.width * cloud.height); print_info (" points]\n");
  print_info ("Available dimensions: "); print_value ("%s\n", pcl::getFieldsList (cloud).c_str ());

  return (true);
}
// 利用pcl::PCDWriter类将PCLPointCloud2格式的点云存储为PCD格式文件，当write类下的write函数中的bool变量format为true时，将被保存为二进制格式，为false时，保存为ASCII码格式
void saveCloud (const std::string &filename, const pcl::PCLPointCloud2 &cloud, bool format)
{
  TicToc tt;
  tt.tic ();

  print_highlight ("Saving "); print_value ("%s ", filename.c_str ());
  
  pcl::PCDWriter writer;
  writer.write (filename, cloud, Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), format);
  
  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", cloud.width * cloud.height); print_info (" points]\n");
}

// main函数主要调用上面两个函数实现PLY文件向PCD文件的转换
int main (int argc, char** argv)
{
  print_info ("Convert a PLY file to PCD format. For more information, use: %s -h\n", argv[0]);

  if (argc < 3)
  {
    printHelp (argc, argv);
    return (-1);
  }

  // Parse the command line arguments for .pcd and .ply files
  std::vector<int> pcd_file_indices = parse_file_extension_argument (argc, argv, ".pcd");
  std::vector<int> ply_file_indices = parse_file_extension_argument (argc, argv, ".ply");
  if (pcd_file_indices.size () != 1 || ply_file_indices.size () != 1)
  {
    print_error ("Need one input PLY file and one output PCD file.\n");
    return (-1);
  }

  // Command line parsing
  bool format = 1;
  parse_argument (argc, argv, "-format", format);
  print_info ("PCD output format: "); print_value ("%s\n", (format ? "binary" : "ascii"));

  // 读取PLY文件
  pcl::PCLPointCloud2 cloud;
  if (!loadCloud (argv[ply_file_indices[0]], cloud)) 
    return (-1);

  // 转换为PCD格式并存储
  saveCloud (argv[pcd_file_indices[0]], cloud, format);

  return (0);
}
```



## 二、点云读取

### 1、数据类型

```
pcl::PointCloud<pcl::PointCloud>
```

是一个基本的数据类型，**PointCloud**是C++里的一个类，包含以下函数：

```
pcl::PointCloud<pcl::PointCloud::width>
pcl::PointCloud<pcl::PointCloud::height>
```

- **width**:在无序数据集中表示点的总数；有序数据集中表示一行中点的个数。

- **height**：无序数据集为常量1；有序数据集中表示总行数。

```
//有序点云
cloud.width = 640; // there are 640 points per line
cloud.height = 480; // thus 640*480=307200 points total in the dataset
//无序点云
cloud.width = 307200;
cloud.height = 1; // unorganized point cloud dataset with 307200 points
```

若需要判断点云是否有序，无需判断height=1，只需用如下函数：

```
if (!cloud.isOrganized ())
```



```
pcl::points<pcl::PointCloud::points>
```

**points**:用于存储PointT类型的向量，例如PointXYZ类型的点

```
pcl::PointCloud<pcl::PointXYZ> cloud;//PointXYZ类型的点储存到点云cloud
std::vector<pcl::PointXYZ> data = cloud.points;//cloud里的数据是储存在points里面的
```



### 2.点的类型

**PointXYZ**成员变量：float x,y,z

- 定义一个点云（指针类型）

  ```
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);//指针类型
  ```

- 访问点的坐标

  ```
  cloud->points[i].x
  pcl::PointXYZ point;//创建一个点放到点云里去
  point.x = 2.0f - y;
  point.y = y;
  point.z = z;
  cloud.points.push_back(point);
  ```

  其中 points是个vector变量，points[i]是单个的点、points[i].data[0]、points[i].x用于访问点的x坐标。



### 3.写入点云数据

```c++
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int main(int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;

    // 写入点云数据
    cloud.width = 5;
    cloud.height = 1;
    cloud.is_dense = false;
    cloud.points.resize(cloud.width * cloud.height);

    for (std::size_t i = 0; i < cloud.points.size(); ++i)
    {
        cloud.points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
        cloud.points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
        cloud.points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
    }

    pcl::io::savePCDFileASCII("test_pcd.pcd", cloud);//储存在pcd文件中
    std::cerr << "Saved " << cloud.points.size() << " data poicnts to test_pcd.pcd." << std::endl;

    for (std::size_t i = 0; i < cloud.points.size(); ++i)
        std::cerr << "    " << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << std::endl;

    return (0);
}
```



### 4、读取点云数据

```c++
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int main(int argc, char** argv)
{
    //创建一个PointCloud <PointXYZ> Boost共享指针并对其进行初始化。
    //从堆中分配了一个PointCloud<pcl::PointXYZ>对象，把这个对象的指针赋值给了cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ>("test_pcd.pcd", *cloud) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file test_pcd.pcd \n");
        return (-1);
    }

    //Loaded 个数 data points from 文件 with the following fields: x y z
    std::cout << "Loaded "
        << cloud->width * cloud->height   //点的个数
        << " data points from test_pcd.pcd with the following fields: "
        << std::endl;

    for (std::size_t i = 0; i < cloud->points.size(); ++i)
        std::cout << "' " << cloud->points[i].x //点云里点的值
        << " " << cloud->points[i].y
        << " " << cloud->points[i].z << std::endl;

    return (0);
}
```



## 三、点云可视化

### 1、简单可视化工具CloudViewer

CloudViewer类是PCL可视化中相对简单的一个点云查看工具，**简单几行代码就可以显示点云**，但是它不能用到多线程程序中。

代码：

```c++
#include<pcl/visualization/cloud_viewer.h>
#include<pcl/io/io.h>
#include<pcl/io/pcd_io.h>//pcd 读写类相关的头文件。
#include<pcl/io/ply_io.h>
#include<pcl/point_types.h> //PCL中支持的点类型头文件。

#include<iostream>//标准C++库中的输入输出类相关头文件。
#include<string>

using namespace std;

void viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
{
	viewer.setBackgroundColor(0, 0, 0);   //设置背景颜色为黑色
}

int main()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);	//创建点云对象

	string filepath = "rabbit_t.pcd";
	if (-1 == pcl::io::loadPCDFile(filepath, *cloud)) //打开点云文件
	{
		cout << "error input!" << endl;
		return -1;
	}

	cout << cloud->points.size() << endl;
	pcl::visualization::CloudViewer viewer("Cloud Viewer");     //创建viewer对象

	viewer.showCloud(cloud);
	while (!viewer.wasStopped())	//判断窗口有没有退出
	{

	}
	return 0;
}
```



### 2、PCLVisualizer可视化工具

#### a、点云的简单显示：

PCLVisualizer可视化类要比CloudViewer类功能强大很多，但是用法也复杂更多

代码：

```c++
#include<pcl/visualization/cloud_viewer.h>
#include<pcl/io/io.h>
#include<pcl/io/pcd_io.h>//pcd 读写类相关的头文件。
#include<pcl/io/ply_io.h>
#include<pcl/point_types.h> //PCL中支持的点类型头文件。

#include<iostream>//标准C++库中的输入输出类相关头文件。
#include<string>

using namespace std;

boost::shared_ptr<pcl::visualization::PCLVisualizer> SimpleVisual(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Simple Viewer"));	//创建一个窗口
	viewer->setBackgroundColor(0, 0, 0);	//设置背景颜色
	viewer->addPointCloud<pcl::PointXYZ>(cloud, "Simple Cloud");	//添加点云
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Simple Cloud");	//设置点云呈现的属性，点的大小，点云对象
	viewer->addCoordinateSystem(1.0);	//添加一个坐标系统，也就是x,y,z三个坐标轴，比例尺设为1.0
	viewer->initCameraParameters();	//初始化相机参数，让用户在默认的角度下观察点云
	return viewer;
}

int main()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	string filepath = "rabbit_t.pcd";
	if (-1 == pcl::io::loadPCDFile(filepath, *cloud)) //打开点云文件
	{
		cout << "error input!" << endl;
		return -1;
	}

	cout << cloud->points.size() << endl;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	viewer = SimpleVisual(cloud);

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);	//100ms刷新一次屏幕
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

	return 0;
}
```



#### b、点云发现显示：

点云的法线是点云非常重要的一个特征，显示点云的法线有助于我们去理解点云的一些情况

代码：

```c++
#include<pcl/visualization/cloud_viewer.h>
#include<pcl/io/io.h>
#include<pcl/io/pcd_io.h>//pcd 读写类相关的头文件。
#include<pcl/io/ply_io.h>
#include<pcl/point_types.h> //PCL中支持的点类型头文件。
#include <pcl/features/normal_3d.h>

#include<iostream>//标准C++库中的输入输出类相关头文件。
#include<string>

#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL);
VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingFreeType);

using namespace std;

//显示点云法线
boost::shared_ptr<pcl::visualization::PCLVisualizer> normalsVis(
	pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals)
{
	//创建3D窗口并添加点云其包括法线  
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	//pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZ> rgb(cloud);
	viewer->addPointCloud<pcl::PointXYZ>(cloud,"sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
	viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, normals, 10, 0.1, "normals");		//这里面中的10是指每10个点显示一个法线，长度为0.1m
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	return viewer;
}

int main()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	string filepath = "Grass.pcd";
	if (-1 == pcl::io::loadPCDFile(filepath, *cloud)) //打开点云文件
	{
		cout << "error input!" << endl;
		return -1;
	}

	//以0.1的半径来获取点云法线
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;	//创建正态估计对象，可以利用他来估计3位表面点的法线、曲率等属性
	ne.setInputCloud(cloud);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());	//创建kdtree对象
	ne.setSearchMethod(tree);	//设置搜索的方式
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>());	//创建法向量点云对象
	ne.setRadiusSearch(0.1);	//设置搜索的半径
	ne.compute(*cloud_normals);

	cout << cloud->points.size() << endl;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	//viewer = SimpleVisual(cloud);
	viewer = normalsVis(cloud, cloud_normals);

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);	//100ms刷新一次屏幕
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

	return 0;
}
```



**但是，点云法向量的计算是比较费时间的。有试过，对一个3万点的点云计算其法线，要耗时几十分钟**

#### c、多个点云显示

PCLVisualizer类也允许我们进行多个点云的显示，有利于我们进行对比

代码：

```c++
#include<pcl/visualization/cloud_viewer.h>
#include<pcl/io/io.h>
#include<pcl/io/pcd_io.h>//pcd 读写类相关的头文件。
#include<pcl/io/ply_io.h>
#include<pcl/point_types.h> //PCL中支持的点类型头文件。
#include <pcl/features/normal_3d.h>

#include<iostream>//标准C++库中的输入输出类相关头文件。
#include<string>

#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL);
VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingFreeType);

using namespace std;

//显示多个点云
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewportsVis(
	pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals1, pcl::PointCloud<pcl::Normal>::ConstPtr normals2)
{
	// 创建3D窗口并添加显示点云其包括法线
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->initCameraParameters();
	int v1(0);
	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer->setBackgroundColor(0, 0, 0, v1);
	viewer->addText("Radius: 0.05", 10, 10, "v1 text", v1);
	//pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	viewer->addPointCloud<pcl::PointXYZ>(cloud,  "sample cloud1", v1);
	int v2(0);
	viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	viewer->setBackgroundColor(0.3, 0.3, 0.3, v2);
	viewer->addText("Radius: 0.1", 10, 10, "v2 text", v2);
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color(cloud, 0, 255, 0);
	viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud2", v2);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud1");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud2");
	viewer->addCoordinateSystem(1.0);

	viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, normals1, 10, 0.2, "normals1", v1);
	viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, normals2, 10, 0.2, "normals2", v2);

	return (viewer);
}

int main()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	string filepath = "Grass.pcd";
	if (-1 == pcl::io::loadPCDFile(filepath, *cloud)) //打开点云文件
	{
		cout << "error input!" << endl;
		return -1;
	}

	//以0.05的半径来获取点云法线
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;	//创建法线估计对象，可以利用他来估计3位表面点的法线、曲率等属性
	ne.setInputCloud(cloud);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());	//创建kdtree对象
	ne.setSearchMethod(tree);	//设置搜索的方式
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>());	//创建法向量点云对象
	ne.setRadiusSearch(0.05);	//设置搜索的半径
	ne.compute(*cloud_normals);

	//  0.1为搜索半径获取点云法线
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2(new pcl::PointCloud<pcl::Normal>());
	ne.setRadiusSearch(0.1);
	ne.compute(*cloud_normals2);

	cout << cloud->points.size() << endl;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	viewer = viewportsVis(cloud, cloud_normals, cloud_normals2);

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);	//100ms刷新一次屏幕
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	
	return 0;
}
```



### 3.pcl_viewer工具

pcl-viewer是常用得pcl可视化工具。可以在Ubuntu系统下，在终端用sudo命令安装：

```
sudo apt-get install  pcl-tools
```

默认安装路径 `usr/bin`

**该路径下有很多有用的可执行程序：`pcl_png2pcd`、`pcl_ply2obj`、`pcl_pcd2png`、`pcl_pcd2ply`等等**

在pcd文件目录下可视化点云地图，输入命令为

```
pcl_viewer simcity9.pcd

// 设置点的大小(点体积)
pcl_viewer 1.pcd -ps 2
```

- 快捷键：

  - r键：重现视角。如果读入文件没有在主窗口显示，不妨按下键盘的r键一试。

  - j键：截图功能，生成的cam和png文件与pcd文件在同一目录下。其中png为截图图片，cam则可再次打开，**不过初始视角为截图时视角**

    - 打开cam文件 

      ```
       pcl_viewer xxx.pcd -cam  xxx.cam
      ```

  - g键：显示/隐藏坐标轴。

  - 放大缩小： 除了滚动滑轮外，还可以有 Alt +/- (每次更小单位的变大变小)

- 一次性打开两个视图

```
pcl_viewer my1.pcd my2.pcd
```

- pcd文件转为ply文件

```
pcd2ply input.pcd output.ply
```

- 将对齐的点云png和RGB图像合成点云pcd
  - 输入：rgb1.png 和 depth.png
  - 输出：points1.pcd

```
pcl_png2pcd --intensity_type UINT_8 rgb1.png depth1.png points1.pcd
```

