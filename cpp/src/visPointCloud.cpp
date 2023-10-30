#include <fstream>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>

// 获取VELODYNE BIN文件中所包含的点数
int getBinSize(std::string path)
{
    int size = 0;
    FILE  *fp = fopen(path.c_str(), "rb");
    if (fp)
    {
        fseek(fp, 0, SEEK_END);
        size = ftell(fp);
        fclose(fp);
    }
    size = size/(int)sizeof(float)/4;
    return size;
}

// 读取KITTI VELODYNE点云
Eigen::MatrixXf readBin(std::string path, int size)
{
    Eigen::MatrixXf pc(size, 4);
    std::ifstream velodyne_bin(path, std::ios::binary);
    for(int i=0; i<size; i++)
    {
        for(int j=0; j<4; j++)
        {
            float data;
            velodyne_bin.read((char*)&data, sizeof(float));
            pc(i,j) = data;
        }
    }
    velodyne_bin.close();
    return pc;
}

int main(int argc, char** argv)
{
    if(argc!=2)
    {
        std::cout<<"argc!=2"<<std::endl;
        return 0;
    }
    argv++;
    std::string file = std::string(*argv);
    int size = getBinSize(file);
    Eigen::MatrixXf pc_eigen = readBin(file, size);
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc(new pcl::PointCloud<pcl::PointXYZI>);
    for(int i=0; i<size; i++)
    {
        pcl::PointXYZI temp;
        temp.x = pc_eigen(i, 0);
        temp.y = pc_eigen(i, 1);
        temp.z = pc_eigen(i, 2);
        pc->push_back(temp);
    }
    pcl::visualization::CloudViewer viewer("cloud view");
    viewer.showCloud(pc);
    while (!viewer.wasStopped())
    {

    }

    return 0;
}