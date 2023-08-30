import numpy as np
from PIL import Image
from matplotlib import pyplot as plt


# 加载calib文件
def loadCalib(path):
    f = open(path, 'r')
    lines = f.readlines()
    calib = {
        "P0": lines[0][4:].strip().split(' '),
        "P1": lines[1][4:].strip().split(' '),
        "P2": lines[2][4:].strip().split(' '),
        "P3": lines[3][4:].strip().split(' '),
        "Tr": lines[4][4:].strip().split(' '),
        "TrRT": lines[4][4:].strip().split(' ')
    }
    for key in calib.keys():
        for i in range(len(calib[key])):
            calib[key][i] = float(calib[key][i])
    for key in calib.keys():
        calib[key] = [calib[key][0:4], calib[key][4:8], calib[key][8:12]]
    calib['TrRT'].append([0, 0, 0, 1])
    return calib


"""
calib_file: calib文件路径
image_index: 需要对齐的相机序号0~3
image_path: png图片文件的路径
lidar_path: velodyne点云文件的路径
"""


def align(calib_file, image_index, image_path, lidar_path):
    # 读取calib文件
    calib = loadCalib(calib_file)
    # 读取图片
    img = np.array(Image.open(image_path)).astype(np.uint8)
    # 读取点云
    points = np.fromfile(lidar_path, dtype=np.float32).reshape(-1, 4)
    # 取相机前方的点
    points = points[points[:, 0] >= 1]
    # 从lidar到相机的变换矩阵
    P_velo_to_img = np.matmul(np.array(calib['P' + str(image_index)]), np.array(calib['TrRT']))
    # 对齐
    points[:, 3] = 1
    points = np.matmul(P_velo_to_img, points.T).T
    points = np.divide(points[:, 0:2], np.array([points[:, 2], points[:, 2]]).T)
    x = points[:, 1].astype(int)
    y = points[:, 0].astype(int)
    sel = (x >= 0) & (x < img.shape[0]) & (y >= 0) & (y < img.shape[1])
    x, y = x[sel], y[sel]
    img[x, y] = [100, 255, 0]
    
    return img


if __name__ == '__main__':
    # example
    calib_path = "/mnt/share_disk/KITTI/dataset/calib/00/calib.txt"
    image_index = 2
    image_path = "/mnt/share_disk/KITTI/dataset/sequences/00/image_2/000000.png"
    lidar_path = "/mnt/share_disk/KITTI/dataset/sequences/00/velodyne/000000.bin"
    img = align(calib_path, image_index, image_path, lidar_path)
    plt.imsave("align.png", img, cmap='jet')