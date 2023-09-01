#include <fstream>
#include <algorithm>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using namespace std;

// 加载calib文件
std::vector<Eigen::Matrix<float, 3, 4>> readCalib(string path)
{
    vector<Eigen::Matrix<float, 3, 4>> calib(5);  // P0 P1 P2 P3 Tr

    std::ifstream file(path);
    string temp_str;
    for(int n=0; n<5; n++)
    {
        file >> temp_str;
        for(int i=0; i<3; i++)
        {
            for(int j=0; j<4; j++)
            {
                file >> temp_str;
                calib[n](i,j) = std::stof(temp_str);
            }
        }
        
    }
    file.close();

    return calib;
}

// 获取VELODYNE BIN文件中所包含的点数
int getBinSize(string path)
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
Eigen::MatrixXf readBin(string path, int size)
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

cv::Mat align(string calib_file, int camera_index, string image_path, string lidar_path)
{
    // 读取calib文件
    std::vector<Eigen::Matrix<float, 3, 4>>  calib = readCalib(calib_file);
    // 读取图片
    cv::Mat img = cv::imread(image_path, 0);
    // 读取点云
    int size = getBinSize(lidar_path);
    Eigen::MatrixXf pc = readBin(lidar_path, size);
    // 从lidar到相机的变换矩阵
    Eigen::Matrix<float, 4, 4> calib_lidar;
    calib_lidar.block<3,4>(0,0) = calib[4];
    calib_lidar.block<1,4>(3,0) = Eigen::Matrix<float, 1, 4>{0,0,0,1};
    Eigen::Matrix<float, 3, 4> P_velo_to_img = calib[camera_index]*calib_lidar;
    // 对齐
    for(int i=0; i<size; i++)
    {
        pc(i, 3) = 1;
    }
    Eigen::MatrixXf pc_align = (P_velo_to_img*pc.transpose()).transpose();
    int height = img.size[0];
    int width = img.size[1];
    for(int i=0; i<size; i++)
    {
        if(pc(i, 0) < 1)
        {
            continue;
        }
        pc_align(i, 0) = pc_align(i, 0)/pc_align(i, 2);
        pc_align(i, 1) = pc_align(i, 1)/pc_align(i, 2);
        int x = pc_align(i, 1);
        int y = pc_align(i, 0);
        if(x < 0 || x > height || y < 0 || y > width)
        {
            continue;
        }
        img.at<uchar>(x, y) = 255;
    }
    return img;
}

int main()
{
    // example
    cv::Mat img = align("calib.txt", 2, "000000.png", "000000.bin");
    cv::imshow("image", img);
    cv::waitKey(0);

    return 0;
}
