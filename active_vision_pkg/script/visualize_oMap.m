num_hid = 26;

% read omap and cmap
omap_path = '../data/sarsop_data/save/online/oMap.txt';
cmap_path = '../data/sarsop_data/save/online/cMap.txt';

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



%% DISPLAY
% % display oMap
% for k = 1:num_hid
%     figure;
%     imagesc(squeeze(oMap(k,:,:)));
% end
% 
% display cMap
% figure;
% imagesc(cMap);

%{
R = dlmread(omap_path);

% normalize
denum = sum(R,2);
R_norm = bsxfun(@rdivide,R,denum);

assert( all(abs(sum(R_norm,2) - 1) < 0.01))
% plot
num_hid = 26;
inc = size(R,1) / num_hid;
num_sta = inc;
num_vp = num_sta - 1;
num_obs = size(R,2);
oMap = zeros(num_hid,num_vp,num_obs);

for k = 1:num_hid
    oMap(k,:,:) = R_norm(((k-1)*inc + 1):(k*inc-1),:);
    figure;
    imagesc(squeeze(oMap(k,:,:)));
end
%}
%{
% Compute D(q_i||q_j)
tic;
D = zeros(num_vp,num_hid,num_hid);
for vp = 1:num_vp
    for i = 1:num_hid
        for j = 1:num_hid
            D(vp,i,j) = 0;
            if i ~= j
                for z = 1:num_obs
                    D(vp,i,j) = D(vp,i,j) + oMap(i,vp,z)*log2( oMap(i,vp,z) / oMap(j,vp,z) );
                end
            end
        end
    end
end
toc;

tic;
num_rand = 100;
vecs = [randfixedsum(num_vp,num_rand,1,0,1),eye(num_vp)];
minval = inf;
minact = -1;
for k = 1:size(vecs,2)
    sumD = zeros(num_hid,num_hid);
    for vp = 1:num_vp
        for i = 1:num_hid
            for j = 1:num_hid
                if i~=j
                    sumD(i,j) = sumD(i,j) + vecs(vp,k)*squeeze(D(vp,i,j));
                end
            end
        end
    end
    sumD = setdiag(sumD,inf);
    new_min = min(min(sumD));
    
    if(new_min < minval)
       minval = new_min;
       minact = k;
    end
end
toc;
%}

