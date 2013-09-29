obj_name = 'watercan';
save_dir = '/media/natanaso/Data/Stuff/Research/git/icra_2013/ns_shared/penn_nbv_slam/active_vision_pkg/data';

[Y,P,R] = meshgrid(0:60:300,0,0:90:270);
Y = Y(:);
P = P(:);
R = R(:);

num_hyp = length(Y);

toWrite = transpose([repmat([obj_name ' '],num_hyp,1), num2str([R,P,Y]), repmat('\n',num_hyp,1)]);
toWrite = transpose(toWrite(:));

% write hypotheses
filename = [save_dir '/hypotheses.txt'];
[fid,Msg] = fopen(filename,'wt');
if fid == -1, error(Msg); end
fprintf(fid,toWrite);
fclose(fid);

quats = angle2quat(Y, P, R);

D = rad2deg(squareform(pdist(quats,@SO3_metric)));