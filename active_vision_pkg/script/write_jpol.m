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
B = zeros(num_hid*(num_hid-1),num_vp);
didx = sub2ind([num_hid,num_hid],1:num_hid,1:num_hid);  % diag indices;
for a = 1:num_vp
   r = transpose(D(a,:));
   r(didx) = [];
   B(:,a) = r;
end

% the policy
P = zeros(num_vp,num_hid+1,num_vp);

% compute for 0 case
for xx = 1:num_vp
    cvec = cMap(xx,:);
    Bmod = bsxfun(@rdivide,B,cvec);
    tic;
    cvx_begin quiet
        variables t v(num_vp);
        maximize( t );
        subject to
        ones(num_hid*(num_hid-1),1)*t - Bmod*v <= 0;
        ones(1,num_vp)*v == 1;
        0 <= v <= 1;
        0 <= t;
    cvx_end    
    toc;
    
    P(xx,1,:) = v;
end

% compute the other M cases
for i_star = 1:num_hid
    start_idx = (i_star-1)*(num_hid-1)+1;
    end_idx = i_star*(num_hid-1);
    BB = B((start_idx:end_idx),:);
    
    for xx = 1:num_vp
        cvec = cMap(xx,:);
        Bmod = bsxfun(@rdivide,BB,cvec);
        
        tic;
        cvx_begin quiet
            variables t v(num_vp);
            maximize( t );
            subject to
            ones((num_hid-1),1)*t - Bmod*v <= 0;
            ones(1,num_vp)*v == 1;
            0 <= v <= 1;
            0 <= t;
        cvx_end    
        toc;
        
        P(xx,i_star+1,:) = v;
    end
end

% write P to file

