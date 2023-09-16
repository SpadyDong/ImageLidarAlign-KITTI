function calib = readCalib(path)

calib = struct();
keys = ["P0", "P1", "P2", "P3", "Tr"];

fid = fopen(path,'r');
for i = 1:5
    [~,~] = fscanf(fid,'%s',1);
    for j = 1:3
        for k = 1:4
            [f,~] = fscanf(fid,'%f',1);
            calib.(keys(i))(j, k) = f;
        end
    end
end
calib.("TrRT") = calib.("Tr");
calib.("TrRT")(4,:) = [0 0 0 1];
fclose(fid);