function WriteBoundaryValues()
global params_
delete('SixBoundaryValues');
fid = fopen('SixBoundaryValues', 'w');
fprintf(fid, '1  %f\r\n', params_.task.x0);
fprintf(fid, '2  %f\r\n', params_.task.y0);
fprintf(fid, '3  %f\r\n', params_.task.theta0);
fprintf(fid, '4  %f\r\n', params_.task.xf);
fprintf(fid, '5  %f\r\n', params_.task.yf);
fprintf(fid, '6  %f\r\n', params_.task.thetaf);
fclose(fid);
end