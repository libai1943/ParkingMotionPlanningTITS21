function V = CreateVehiclePolygon(x, y, theta, resolution)
global params_
cos_theta = cos(theta);
sin_theta = sin(theta);
vehicle_half_width = params_.vehicle.lb * 0.5;
AX = x + (params_.vehicle.lf + params_.vehicle.lw) * cos_theta - vehicle_half_width * sin_theta;
BX = x + (params_.vehicle.lf + params_.vehicle.lw) * cos_theta + vehicle_half_width * sin_theta;
CX = x - params_.vehicle.lr * cos_theta + vehicle_half_width * sin_theta;
DX = x - params_.vehicle.lr * cos_theta - vehicle_half_width * sin_theta;
AY = y + (params_.vehicle.lf + params_.vehicle.lw) * sin_theta + vehicle_half_width * cos_theta;
BY = y + (params_.vehicle.lf + params_.vehicle.lw) * sin_theta - vehicle_half_width * cos_theta;
CY = y - params_.vehicle.lr * sin_theta - vehicle_half_width * cos_theta;
DY = y - params_.vehicle.lr * sin_theta + vehicle_half_width * cos_theta;
V.x = [AX, linspace(AX, BX, resolution), BX, linspace(BX, CX, resolution), CX, linspace(CX, DX, resolution), DX, linspace(DX, AX, resolution)];
V.y = [AY, linspace(AY, BY, resolution), BY, linspace(BY, CY, resolution), CY, linspace(CY, DY, resolution), DY, linspace(DY, AY, resolution)];
end