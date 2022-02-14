function [ptrch]=Dessin_Robot(Q,ptrch,dimensions)


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



A0=quat_mult(quat_mult(alphaZ_conj,[0 A0_S]),alphaZ);
A1=quat_mult(quat_mult(alphaZ_conj,[0 A1_S]),alphaZ);
A2=quat_mult(quat_mult(alphaZ_conj,[0 A2_S]),alphaZ);
A3=quat_mult(quat_mult(alphaZ_conj,[0 A3_S]),alphaZ);

B0=quat_mult(quat_mult(alphaZ_conj,[0 B0_S]),alphaZ);
B1=quat_mult(quat_mult(alphaZ_conj,[0 B1_S]),alphaZ);
B2=quat_mult(quat_mult(alphaZ_conj,[0 B2_S]),alphaZ);
B3=quat_mult(quat_mult(alphaZ_conj,[0 B3_S]),alphaZ);


vertex_matrix=[A0(2:4);A1(2:4);A2(2:4);A3(2:4);  B0(2:4);B1(2:4);B2(2:4);B3(2:4)];
set(ptrch,'Vertices',vertex_matrix);

