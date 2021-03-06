% ==============================================================================
%  MATLAB Source Codes for "Optimization-based trajectory planning for
%  autonomous parking with irregularly placed obstacles: A lightweight
%  iterative framework".
% ==============================================================================
%   Copyright (C) 2022 Bai Li
%   Users should cite the following article when utilizing these source codes.
%   Bai Li, Tankut Acarman, Youmin Zhang, et al., “Optimization-based
%   trajectory planning for autonomous parking with irregularly placed
%   obstacles: A lightweight iterative framework,” IEEE Transactions on
%   Intelligent Transportation Systems, accepted on Aug. 27, 2021.
% ==============================================================================
%   2022.04.24
% ==============================================================================
close all; clc; clear global params_; clear all;
global params_
for ii = 1 : 115
    params_.user.case_id = ii;
    InitializeParams();
    SearchTrajectoryViaFTHA();
    OptimizeTrajectoryViaLIOM();
end