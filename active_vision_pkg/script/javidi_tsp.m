num_hid = 7;


% read omap and cmap
omap_path = '../data/sarsop_data/oMap.txt';
cmap_path = '../data/sarsop_data/cMap.txt';

oMap_tmp = dlmread(omap_path);  %num_hid*(num_vp+1) x num_obs
cMap_tmp = dlmread(cmap_path);  %num_hid*(num_vp+num_hid) x (num_vp+1)

% normalize oMap
denum = sum(oMap_tmp,2);
oMap_tmp_norm = bsxfun(@rdivide,oMap_tmp,denum);

assert( all(abs(sum(oMap_tmp_norm,2) - 1) < 0.01));
num_sta = size(oMap_tmp,1) / num_hid;
num_vp = num_sta - 1;
num_obs = size(oMap_tmp,2);
oMap = zeros(num_hid,num_vp,num_obs);

for k = 1:num_hid
    oMap(k,:,:) = oMap_tmp_norm(((k-1)*num_sta + 1):(k*num_sta-1),:);
end

% get cMap for vps only and subtract measurement cost
cMap = cMap_tmp(1:num_vp,1:num_vp);
cMap = cMap - cMap(1,1);


%% DISPLAY
% % display oMap
% for k = 1:num_hid
%     figure;
%     imagesc(squeeze(oMap(k,:,:)));
% end

% display cMap
% figure;
% imagesc(cMap);

%% Compute D(q_i||q_j)
tic;
D = zeros(num_vp,num_hid,num_hid);
for vp = 1:num_vp
    for i = 1:num_hid
        for j = 1:num_hid
            if i ~= j
                for z = 1:num_obs
                    D(vp,i,j) = D(vp,i,j) + oMap(i,vp,z)*log2( oMap(i,vp,z) / oMap(j,vp,z) );
                end
            end
        end
    end
end
toc;

%% Compute Javidi Policy using LP
% num_hid = M = number of hypotheses
% num_vp = |X| = number of viewpoints
R = zeros(num_hid*(num_hid-1),num_vp);
didx = sub2ind([num_hid,num_hid],1:num_hid,1:num_hid);  % diag indices;
for a = 1:num_vp
   r = transpose(squeeze(D(a,:,:)));
   r = r(:);
   r(didx) = [];
   R(:,a) = r;
end

tic;
cvx_begin
    variables t v(num_vp);
    maximize( t );
    subject to
    ones(num_hid*(num_hid-1),1)*t - R*v <= 0;
    ones(1,num_vp)*v == 1;
    0 <= v <= 1;
    0 <= t;
cvx_end    
toc;

% Display results
disp('------------------------------------------------------------------------');
disp('The optimal t obtained is');
disp(t);
disp('The optimal v obtained is');
disp(v);

%% Compute TSP (solve for all initial states and save!)
optRoutes = cell(num_vp,1);
minDists = zeros(num_vp,1);


% Genetic Algorithm Parameters
popSize = 60;
numIter = 1e3;
showProg = 0;
showResult = 0;

tic;
for v_start = 1:num_vp
    vps_idx = v > 0;
    vps_idx(v_start) = false;
    vps_id = transpose(1:num_vp);
    vps_id = [v_start; vps_id(vps_idx)];
    xy = [vps_id, vps_id];
    cMap_reduced = cMap(vps_id,vps_id);
    [optRoute,minDist] = tspofs_ga(xy,cMap_reduced,popSize,numIter,showProg,showResult);
    optRoutes{v_start} = [xy(1,1);xy(optRoute,1)];
    minDists(v_start) = minDist;
end
toc;


%% Compute Javidi Policy for a specific i
V = zeros(num_hid,num_vp);
T = zeros(num_hid,1);
optRoutes_i = cell(num_hid,num_vp);
minDists_i = zeros(num_hid,num_vp);
for i_star = 1:num_hid
    start_idx = (i_star-1)*(num_hid-1)+1;
    end_idx = i_star*(num_hid-1);
    tic;
    cvx_begin
        variables t v(num_vp);
        maximize( t );
        subject to
        ones((num_hid-1),1)*t - R((start_idx:end_idx),:)*v <= 0;
        ones(1,num_vp)*v == 1;
        0 <= v <= 1;
        0 <= t;
    cvx_end    
    toc;
    V(i_star,:) = v;
    T(i_star) = t;
    
    % TSP
    tic;
    for v_start = 1:num_vp
        vps_idx = v > 0;
        vps_idx(v_start) = false;
        vps_id = transpose(1:num_vp);
        vps_id = [v_start; vps_id(vps_idx)];
        xy = [vps_id, vps_id];
        cMap_reduced = cMap(vps_id,vps_id);
        [optRoute,minDist] = tspofs_ga(xy,cMap_reduced,popSize,numIter,showProg,showResult);
        optRoutes_i{i_star,v_start} = [xy(1,1);xy(optRoute,1)];
        minDists_i(i_star,v_start) = minDist;
    end
    toc;
end

% display
figure;
imagesc(V);

