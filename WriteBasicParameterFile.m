function WriteBasicParameterFile()
global params_
delete('BasicParameters');
fid = fopen('BasicParameters', 'w');
fprintf(fid, '1 %g\r\n', params_.scenario.xmin);
fprintf(fid, '2 %f\r\n', params_.scenario.xmax);
fprintf(fid, '3 %f\r\n', params_.scenario.ymin);
fprintf(fid, '4 %f\r\n', params_.scenario.ymax);
fprintf(fid, '5 %f\r\n', params_.vehicle.lw);
fprintf(fid, '6 %f\r\n', params_.vehicle.lf);
fprintf(fid, '7 %f\r\n', params_.vehicle.lr);
fprintf(fid, '8 %g\r\n', params_.vehicle.lb);
fprintf(fid, '9 %f\r\n', params_.vehicle.dual_disk_radius);
fprintf(fid, '10 %f\r\n', params_.vehicle.r2p);
fprintf(fid, '11 %f\r\n', params_.vehicle.f2p);
fprintf(fid, '12 %f\r\n', params_.vehicle.vmax);
fprintf(fid, '13 %f\r\n', params_.vehicle.amax);
fprintf(fid, '14 %f\r\n', params_.vehicle.phymax);
fprintf(fid, '15 %f\r\n', params_.vehicle.wmax);
fprintf(fid, '16 %f\r\n', params_.opti.nfe);
fprintf(fid, '17 %f\r\n', params_.opti.cost_function_external_penalty_weight);
fprintf(fid, '18 %f\r\n', params_.opti.cost_a);
fprintf(fid, '19 %f\r\n', params_.opti.cost_w);
fprintf(fid, '20 %f\r\n', params_.opti.cost_phy);
fclose(fid);
end