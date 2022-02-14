function plotCube3Angles_2(roll, pitch, yaw)
 
Dim_X = 0.5; %size of the cube in X
Dim_Y = 0.5; %size of the cube in Y
Dim_Z = 0.5; %size in Z axes
vertex_matrix = [0 0 0;
1 0 0;
1 1 0;
0 1 0;
0 0 1;
1 0 1;
1 1 1;
0 1 1];
% vertex_matrix = [0 0 0;
% 4 0 0;
% 4 3 0;
% 0 3 0;
% 0 0 1;
% 4 0 1;
% 4 3 1;
% 0 3 1];
 
faces_matrix = [1 2 6 5
2 3 7 6
3 4 8 7
4 1 5 8
1 2 3 4
5 6 7 8];
CubeCenter_Coord = [Dim_X/2 Dim_Y/2 Dim_Z/2];
%origin = CubeCenter;
origin = [0 0 0]; %offset
CubeParameters = [vertex_matrix(:,1)*Dim_X+origin(1),vertex_matrix(:,2)*Dim_Y+origin(2),vertex_matrix(:,3)*Dim_Z+origin(3)];
axis equal;
cube = patch('Vertices',CubeParameters,'Faces',faces_matrix,'FaceColor', 'blue');
rotate(cube, [1,0,0], roll); %rotate cube, axis direction, angle value
rotate(cube,[0,1,0], pitch);
rotate(cube, [0, 0, 1], -yaw);
view(3); %3D
end