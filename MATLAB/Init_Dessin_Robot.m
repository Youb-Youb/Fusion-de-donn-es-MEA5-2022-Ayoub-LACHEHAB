function [ptrch,A0_S,B1_S,B2_S,B3_S]=Init_Dessin_Robot(Q,dimensions)

alphaZ=Q;
alphaZ_conj=quat_conj(Q);


L=dimensions(1);l=dimensions(2);h=dimensions(3);

%J NON diag
A0_S=[L/2 l/2 h/2];
A1_S=[L/2 l/2 -h/2];
A2_S=[L/2 -l/2 -h/2];
A3_S=[L/2 -l/2 h/2];
B0_S=[-L/2 l/2 h/2];
B1_S=[-L/2 l/2 -h/2];
B2_S=[-L/2 -l/2 -h/2];
B3_S=[-L/2 -l/2 h/2];
% B1_S=[-L/2 l/2 -h/2];
% B2_S=[-L/2 -l/2 -h/2];
% B3_S=[-L/2 0 h];



A0=quat_mult(quat_mult(alphaZ_conj,[0 A0_S]),alphaZ);
A1=quat_mult(quat_mult(alphaZ_conj,[0 A1_S]),alphaZ);
A2=quat_mult(quat_mult(alphaZ_conj,[0 A2_S]),alphaZ);
A3=quat_mult(quat_mult(alphaZ_conj,[0 A3_S]),alphaZ);

B0=quat_mult(quat_mult(alphaZ_conj,[0 B0_S]),alphaZ);
B1=quat_mult(quat_mult(alphaZ_conj,[0 B1_S]),alphaZ);
B2=quat_mult(quat_mult(alphaZ_conj,[0 B2_S]),alphaZ);
B3=quat_mult(quat_mult(alphaZ_conj,[0 B3_S]),alphaZ);
vertex_matrix=[A0(2:4);A1(2:4);A2(2:4);A3(2:4);  B0(2:4);B1(2:4);B2(2:4);B3(2:4)];
faces_matrix=[1 2 3 4;1 4 8 5;1 2 6 5;2 3 7 6;3 4 8 7;5 6 7 8];

FC=[1 0 0;0 1 0;0 0 1;1 1 0;0 1 1;1 1 1];
ptrch=patch('Vertices',vertex_matrix,'Faces',faces_matrix,'facecolor','non',...
    'facealpha',1','FaceVertexCData',FC);
    


% 
% 
% alphaZ_conj=quatconj(alphaZ);
% 
% A1_S=[0 L/2 l/2 h/2];
% A1=quat_mult(quat_mult(alphaZ_conj,A1_S),alphaZ)+[0 dec'];
% A2_S=[0 L/2 -l/2 h/2];
% A2=quat_mult(quat_mult(alphaZ_conj,A2_S),alphaZ)+[0 dec'];
% A3_S=[0 L/2 -l/2 -h/2];
% A3=quat_mult(quat_mult(alphaZ_conj,A3_S),alphaZ)+[0 dec'];
% A4_S=[0 L/2 l/2 -h/2];
% A4=quat_mult(quat_mult(alphaZ_conj,A4_S),alphaZ)+[0 dec'];
% 
% B1_S=[0 -L/2 l/2 h/2];
% B1=quat_mult(quat_mult(alphaZ_conj,B1_S),alphaZ)+[0 dec'];
% B2_S=[0 -L/2 -l/2 h/2];
% B2=quat_mult(quat_mult(alphaZ_conj,B2_S),alphaZ)+[0 dec'];
% B3_S=[0 -L/2 -l/2 -h/2];
% B3=quat_mult(quat_mult(alphaZ_conj,B3_S),alphaZ)+[0 dec'];
% B4_S=[0 -L/2 l/2 -h/2];
% B4=quat_mult(quat_mult(alphaZ_conj,B4_S),alphaZ)+[0 dec'];
% vertex_matrix=[A1(2:4);A2(2:4);A3(2:4);A4(2:4);B1(2:4);B2(2:4);B3(2:4);B4(2:4)];
% faces_matrix=[1 2 3 4;1 2 6 5;1 4 8 5;2 3 7 6;5 6 7 8;3 4 8 7];
% ptrch=patch('Vertices',vertex_matrix,'Faces',faces_matrix,'Facecolor','none','edgecolor',couleur, 'erasemode','background','LineWidth',2);
% %ptrch=patch('Vertices',vertex_matrix,'Faces',faces_matrix,'Facecolor','none','edgecolor',couleur, 'erasemode','background','LineWidth',1);
% 
% Up=(A1+B1+A2+B2)/4;
% Front=(A1+A3+A2+A4)/4; 
% Side=(A1+B1+B4+A4)/4;
%  ptrchfront(1)=plot3(Up(2),Up(3),Up(4),'.b','markersize',15);
%  ptrchfront(2)=plot3(Front(2),Front(3),Front(4),'.r','markersize',15);
%  ptrchfront(3)=plot3(Side(2),Side(3),Side(4),'.g','markersize',15);
