
fileID = fopen('../../factors_saving_pkg/logs/factors.txt', 'r');

if fileID == -1
    error('File cannot be opened. Check the path!');
end

formatSpec = '%f %f %f %f %f %f %d';

dataArray = textscan(fileID, formatSpec);

fclose(fileID);

trace_bound = dataArray{1,3};
eig_x_bound = dataArray{1,4};
eig_y_bound = dataArray{1,5};
eig_z_bound = dataArray{1,6};
points_bound = dataArray{1,7};

eig_plane = [eig_x_bound eig_y_bound];
sum_y = 0;
cnty = 0;
sum_x = 0;
cntx=0;
for i=1:size(eig_plane,1)
    if eig_plane(i,1) > eig_plane(i,2)
        sum_y = sum_y+eig_plane(i,2);
        cnty = cnty+1;
    else
        sum_x = sum_x+eig_plane(i,1);
        cntx = cntx+1;
    end
end

avg_x = sum_x/cntx;
avg_y = sum_y/cnty;

avg_z = mean(eig_z_bound);

avg_points = mean(points_bound);
avg_trace = mean(trace_bound);
