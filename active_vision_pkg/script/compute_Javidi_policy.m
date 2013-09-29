% read
omap_path = '../data/sarsop_data/oMap.txt';

R = dlmread(omap_path);

% normalize
denum = sum(R,2);
R_norm = bsxfun(@rdivide,R,denum);

assert( all(abs(sum(R_norm,2) - 1) < 0.01))
% plot
num_hid = 7;
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
maxval = -inf;
maxact = -1;
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
    
    if(new_min > maxval)
       maxval = new_min;
       maxact = k;
    end
end
[~,bestact] = max(vecs(:,maxact));
toc;

%% 
tic;
for k = 1:num_vp
    D(k,:,:) = setdiag(squeeze(D(k,:,:)),10000);
end

F = zeros(num_hid, num_hid);
cvx_begin
    variables t q(num_vp);
    maximize( t );
    subject to
        for i = 1:num_vp
            F = F + squeeze(D(i,:,:))*q(i);
        end
        F-ones(num_hid,num_hid)*t >= 0;
        t >= 0;
        q >= 0;
        q <= 1;
        ones(1,num_vp)*q == 1;
cvx_end
toc;
% Display results
disp('------------------------------------------------------------------------');
disp('The optimal t obtained is');
disp(t);
disp('The optimal q obtained is');
disp(q);

tic;
hid_star = 3;
F = zeros(num_hid,1);

cvx_begin
    variables t q(num_vp);
    maximize( t );
    subject to
        for i = 1:num_vp
            F = F + squeeze(D(i,hid_star,:))*q(i);
        end
        F-ones(num_hid,1)*t >= 0;
        t >= 0;
        q >= 0;
        q <= 1;
        ones(1,num_vp)*q == 1;
cvx_end
toc;
% Display results
disp('------------------------------------------------------------------------');
disp('The optimal t obtained is');
disp(t);
disp('The optimal q obtained is');
disp(q);

%% Simplified LP
% num_hid = M = number of hypotheses
% num_vp = |X| = number of viewpoints
G = zeros(num_hid*(num_hid-1),num_vp);
didx = sub2ind([num_hid,num_hid],1:num_hid,1:num_hid);  % diag indices;
for a = 1:num_vp
   r = transpose(D(a,:));
   r(didx) = [];
   G(:,a) = r;
end

tic;
cvx_begin
    variables t v(num_vp);
    maximize( t );
    subject to
    ones(num_hid*(num_hid-1),1)*t - G*v <= 0;
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