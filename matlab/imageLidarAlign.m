function image = imageLidarAlign(calibPath, imagePath, cameraIndex, lidarPath)
    % 读取calib文件
    calib = readCalib(calibPath);
    % 读取图片
    image = imread(imagePath);
    % 读取点云
    lidar = fopen(lidarPath,'rb');
    pc = fread(lidar,[4 inf],'single')';
    fclose(lidar);
    % 取相机前方的点
    idx = pc(:,1) < 1;
    pc(idx,:) = [];
    % 从lidar到相机的变换矩阵
    P_velo_to_img = calib.("P"+cameraIndex)*calib.TrRT;
    % 对齐
    pc(:,4) = 1;
    pc = (P_velo_to_img*pc')';
    pc = pc(:,1:2)./(pc(:,3)*ones(1,2));
    pc = floor(pc);
    sel = (pc(:,1) <= 0) | (pc(:,1) > size(image, 2)) | (pc(:,2) <= 0) | (pc(:,2) > size(image, 1));
    pc(sel, :) = [];
    d = pc(:,1).*pc(:,1) + pc(:,2).*pc(:,2);
    d = d.^0.5;
    for i=1:size(d)
        image(pc(i,2), pc(i,1)) = d(i);
    end
end

