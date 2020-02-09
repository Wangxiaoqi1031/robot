clc
clear all
syms a1 a2 a3 d4 d5 d6 q1 q2 q3 q4 q5 q6 b lx mx nx px ly my ny py lz mz nz pz x3
%D-H parameter
a1=0.25;a2=0.95;a3=0.3;d4=1.55;d5=0.114;d6=0.123;b=60; 
%Randomly select a set of joint variables
q1_i=14;
q2_i=29.7;
q3_i=-45;
q4_i=71;
q5_i=-63;
q6_i=100;
%Positive kinematics
sb=sind(b);
cb=cosd(b);
A1=[cosd(q1_i) 0 sind(q1_i) a1*cosd(q1_i);sind(q1_i) 0 -cosd(q1_i) a1*sind(q1_i); 0 1 0 0;0 0 0 1];
A2=[cosd(q2_i) -sind(q2_i) 0 a2*cosd(q2_i);sind(q2_i) cosd(q2_i) 0 a2*sind(q2_i); 0 0 1 0;0 0 0 1];
A3=[cosd(q3_i) 0 sind(q3_i) a3*cosd(q3_i);sind(q3_i) 0 -cosd(q3_i) a3*sind(q3_i); 0 1 0 0;0 0 0 1];
A4=[cosd(q4_i) -sind(q4_i)*cosd(b) sind(q4_i)*sind(b) 0;sind(q4_i) cosd(q4_i)*cosd(b) -cosd(q4_i)*sind(b) 0; 0 sind(b) cosd(b) d4;0 0 0 1];
A5=[cosd(q5_i) -sind(q5_i)*cosd(b) -sind(q5_i)*sind(b) 0;sind(q5_i) cosd(q5_i)*cosd(b) cosd(q5_i)*sind(b) 0; 0 -sind(b) cosd(b) d5;0 0 0 1];
A6=[cosd(q6_i) -sind(q6_i) 0 0;sind(q6_i) cosd(q6_i) 0 0; 0 0 1 d6;0 0 0 1];
A= A1*A2*A3*A4*A5*A6;
vpa(A,8)
lx=A(1,1);mx=A(1,2);nx=A(1,3);px=A(1,4);
ly=A(2,1);my=A(2,2);ny=A(2,3);py=A(2,4);
lz=A(3,1);mz=A(3,2);nz=A(3,3);pz=A(3,4);
%Inverse kinematics
%Omit complex calculations,directly calculate elements of A, B, and C matrices
k11_1=2*a2*cb*d5+ 2*a2*d4;
k11_2=2*a2*a3;
k11_3=(d4 + cb*d5)^2 - (px - d6*nx)^2 - (py - d6*ny)^2 - (pz - d6*nz)^2 - a1^2 + a2^2 + a3^2 + d5^2*sb^2 + (2*a1*(px - d6*nx)*(cb*sb*(px - d6*nx) - d5*nx*sb) + 2*a1*(py - d6*ny)*(cb*sb*(py - d6*ny) - d5*ny*sb))/(nx*py - ny*px) + (cb*sb*(2*a1*(px - d6*nx)^2 + 2*a1*(py - d6*ny)^2))/(nx*py - ny*px) ;
k13_1=2*a2*d4 +2*a2*cb*d5;
k13_2=2*a2*a3;
k13_3=(d4 + cb*d5)^2 - (px - d6*nx)^2 - (py - d6*ny)^2 - (pz - d6*nz)^2 - a1^2 + a2^2 + a3^2 + d5^2*sb^2 + (2*a1*(px - d6*nx)*(cb*sb*(px - d6*nx) - d5*nx*sb) + 2*a1*(py - d6*ny)*(cb*sb*(py - d6*ny) - d5*ny*sb))/(nx*py - ny*px) - (cb*sb*(2*a1*(px - d6*nx)^2 + 2*a1*(py - d6*ny)^2))/(nx*py - ny*px);
k14_2=4*a2*d5*sb;
k14_3=4*a3*d5*sb;
k15_3=-(4*sb*(2*a1*(px - d6*nx)^2 + 2*a1*(py - d6*ny)^2))/(nx*py - ny*px);
k16_2=4*a2*d5*sb;
k16_3=4*a3*d5*sb;
k17_1=2*a2*d4+2*a2*cb*d5;
k17_2=2*a2*a3;
k17_3=(d4 + cb*d5)^2 - (px - d6*nx)^2 - (py - d6*ny)^2 - (pz - d6*nz)^2 - a1^2 + a2^2 + a3^2 + d5^2*sb^2 - (2*a1*(px - d6*nx)*(cb*sb*(px - d6*nx) - d5*nx*sb) + 2*a1*(py - d6*ny)*(cb*sb*(py - d6*ny) - d5*ny*sb))/(nx*py - ny*px) - (cb*sb*(2*a1*(px - d6*nx)^2 + 2*a1*(py - d6*ny)^2))/(nx*py - ny*px);
k19_1=2*a2*d4+2*a2*cb*d5;
k19_2=2*a2*a3;
k19_3=(d4 + cb*d5)^2 - (px - d6*nx)^2 - (py - d6*ny)^2 - (pz - d6*nz)^2 - a1^2 + a2^2 + a3^2 + d5^2*sb^2 - (2*a1*(px - d6*nx)*(cb*sb*(px - d6*nx) - d5*nx*sb) + 2*a1*(py - d6*ny)*(cb*sb*(py - d6*ny) - d5*ny*sb))/(nx*py - ny*px) + (cb*sb*(2*a1*(px - d6*nx)^2 + 2*a1*(py - d6*ny)^2))/(nx*py - ny*px) ;
k21_1= - a2*sb^2 + a2*cb^2;
k21_3= cb*d5 - nx*px - ny*py - nz*pz + cb^2*d4 + d6*nx^2 + d6*ny^2 + d6*nz^2 - d4*sb^2 + (a1*nx*(cb*sb*(px - d6*nx) - d5*nx*sb) + a1*ny*(cb*sb*(py - d6*ny) - d5*ny*sb))/(nx*py - ny*px) + (cb*sb*(a1*nx*(px - d6*nx) + a1*ny*(py - d6*ny)))/(nx*py - ny*px);
k22_2=2*a2*sb;
k22_3=2*a3*sb;
k23_1= a2*sb^2 + a2*cb^2;
k23_3= cb*d5 - nx*px - ny*py - nz*pz + cb^2*d4 + d6*nx^2 + d6*ny^2 + d6*nz^2 + d4*sb^2 + (a1*nx*(cb*sb*(px - d6*nx) - d5*nx*sb) + a1*ny*(cb*sb*(py - d6*ny) - d5*ny*sb))/(nx*py - ny*px) - (cb*sb*(a1*nx*(px - d6*nx) + a1*ny*(py - d6*ny)))/(nx*py - ny*px);
k24_2= 4*a2*cb*sb;
k24_3=4*a3*cb*sb;
k25_3=-(4*sb*(a1*nx*(px - d6*nx) + a1*ny*(py - d6*ny)))/(nx*py - ny*px);
k27_1=- a2*sb^2 + a2*cb^2;
k27_3= cb*d5 - nx*px - ny*py - nz*pz + cb^2*d4 + d6*nx^2 + d6*ny^2 + d6*nz^2 - d4*sb^2 - (a1*nx*(cb*sb*(px - d6*nx) - d5*nx*sb) + a1*ny*(cb*sb*(py - d6*ny) - d5*ny*sb))/(nx*py - ny*px) - (cb*sb*(a1*nx*(px - d6*nx) + a1*ny*(py - d6*ny)))/(nx*py - ny*px);
k28_2=-2*a2*sb;
k28_3=-2*a3*sb;
k29_1= a2*sb^2 + a2*cb^2;
k29_3= cb*d5 - nx*px - ny*py - nz*pz + cb^2*d4 + d6*nx^2 + d6*ny^2 + d6*nz^2 + d4*sb^2 - (a1*nx*(cb*sb*(px - d6*nx) - d5*nx*sb) + a1*ny*(cb*sb*(py - d6*ny) - d5*ny*sb))/(nx*py - ny*px) + (cb*sb*(a1*nx*(px - d6*nx) + a1*ny*(py - d6*ny)))/(nx*py - ny*px);
k31_2= a2*cb^2 - a2*sb^2;
k31_3= a3*cb^2 - a1*nz - a3*sb^2 - ((cb*sb*(px - d6*nx) - d5*nx*sb)*(nx*pz - nz*px) + (cb*sb*(py - d6*ny) - d5*ny*sb)*(ny*pz - nz*py))/(nx*py - ny*px) - (cb*sb*((px - d6*nx)*(nx*pz - nz*px) + (py - d6*ny)*(ny*pz - nz*py)))/(nx*py - ny*px);
k32_1= - 2*a2*sb;
k32_3=- 2*d4*sb - 2*cb*d5*sb;
k33_2= a2*cb^2 + a2*sb^2;
k33_3= a3*cb^2 - a1*nz + a3*sb^2 - ((cb*sb*(px - d6*nx) - d5*nx*sb)*(nx*pz - nz*px) + (cb*sb*(py - d6*ny) - d5*ny*sb)*(ny*pz - nz*py))/(nx*py - ny*px) + (cb*sb*((px - d6*nx)*(nx*pz - nz*px) + (py - d6*ny)*(ny*pz - nz*py)))/(nx*py - ny*px);
k34_1= - 4*a2*cb*sb;
k34_3=- 2*d5*sb - 4*cb*d4*sb;
k35_3=(4*sb*((px - d6*nx)*(nx*pz - nz*px) + (py - d6*ny)*(ny*pz - nz*py)))/(nx*py - ny*px);
k36_3=2*d5*sb;
k37_2= a2*cb^2 - a2*sb^2;
k37_3= a3*cb^2 - a1*nz - a3*sb^2 + ((cb*sb*(px - d6*nx) - d5*nx*sb)*(nx*pz - nz*px) + (cb*sb*(py - d6*ny) - d5*ny*sb)*(ny*pz - nz*py))/(nx*py - ny*px) + (cb*sb*((px - d6*nx)*(nx*pz - nz*px) + (py - d6*ny)*(ny*pz - nz*py)))/(nx*py - ny*px);
k38_1= 2*a2*sb;
k38_3=2*d4*sb + 2*cb*d5*sb;
k39_2= a2*cb^2 + a2*sb^2;
k39_3= a3*cb^2 - a1*nz + a3*sb^2 + ((cb*sb*(px - d6*nx) - d5*nx*sb)*(nx*pz - nz*px) + (cb*sb*(py - d6*ny) - d5*ny*sb)*(ny*pz - nz*py))/(nx*py - ny*px) - (cb*sb*((px - d6*nx)*(nx*pz - nz*px) + (py - d6*ny)*(ny*pz - nz*py)))/(nx*py - ny*px);
k41_1=2*d5*sb*a2 *sb^2- 2*d5*sb*a2*cb^2 +4*cb*sb*a2*d4 + 4*cb*sb*a2*cb*d5;
k41_2=4*cb*sb*a2*a3;
k41_3=2*a1*nx*(py - d6*ny) - 2*a1*ny*(px - d6*nx) + 2*d5*sb*d4*sb^2 - 2*d5*sb*(cb*d5 - nx*px - ny*py - nz*pz + cb^2*d4 + d6*nx^2 + d6*ny^2 + d6*nz^2) + 2*cb*sb*((d4 + cb*d5)^2 - (px - d6*nx)^2 - (py - d6*ny)^2 - (pz - d6*nz)^2 - a1^2 + a2^2 + a3^2 + d5^2*sb^2) ;
k42_2=-4*d5*sb*a2*sb;
k42_3=-4*d5*sb*a3*sb;
k43_1= -2*d5*sb*a2*sb^2 - 2*d5*sb*a2*cb^2;
k43_3=2*a1*nx*(py - d6*ny) - 2*a1*ny*(px - d6*nx) - 2*d5*sb*d4*sb^2 - 2*d5*sb*(cb*d5 - nx*px - ny*py - nz*pz + cb^2*d4 + d6*nx^2 + d6*ny^2 + d6*nz^2);
k45_1= -4*sb*(2*a2*d4 + 2*a2*cb*d5);
k45_2=-8*sb*a2*a3;
k45_3=-4*sb*((d4 + cb*d5)^2 - (px - d6*nx)^2 - (py - d6*ny)^2 - (pz - d6*nz)^2 - a1^2 + a2^2 + a3^2 + d5^2*sb^2);
k47_1= - 2*d5*sb*a2*sb^2+ 2*d5*sb*a2*cb^2 - 4*cb*sb*a2*d4 - 4*cb*sb*a2*cb*d5;
k47_2=- 4*cb*sb*a2*a3;
k47_3=2*a1*nx*(py - d6*ny) - 2*a1*ny*(px - d6*nx) - 2*d5*sb*d4*sb^2+ 2*d5*sb*(cb*d5 - nx*px - ny*py - nz*pz + cb^2*d4 + d6*nx^2 + d6*ny^2 + d6*nz^2) - 2*cb*sb*((d4 + cb*d5)^2 - (px - d6*nx)^2 - (py - d6*ny)^2 - (pz - d6*nz)^2 - a1^2 + a2^2 + a3^2 + d5^2*sb^2 );
k48_2= -4*d5*sb*a2*sb;
k48_3=-4*d5*sb*a3*sb;
k49_1= 2*d5*sb*a2*sb^2+2*d5*sb*a2*cb^2;
k49_3=2*a1*nx*(py - d6*ny) - 2*a1*ny*(px - d6*nx) + 2*d5*sb*d4*sb^2 + 2*d5*sb*(cb*d5 - nx*px - ny*py - nz*pz + cb^2*d4 + d6*nx^2 + d6*ny^2 + d6*nz^2);
k51_1=-(a1*sb^2)/2-a3*nz+(a1*cb^2)/2-(a2^2*cb^2)/(2*a1)+(a3^2*cb^2)/(2*a1)-(cb^2*d4^2)/(2*a1)-(cb^4*d5^2)/(2*a1)-(cb^2*px^2)/(2*a1)-(cb^2*py^2)/(2*a1)-(cb^2*pz^2)/(2*a1)+(a2^2*sb^2)/(2*a1)-(a3^2*sb^2)/(2*a1)+(d4^2*sb^2)/(2*a1)-(d5^2*sb^4)/(2*a1)+(px^2*sb^2)/(2*a1)+(py^2*sb^2)/(2*a1)+(pz^2*sb^2)/(2*a1)-(cb^3*d4*d5)/a1-(d4*d6*nx^2)/a1-(d4*d6*ny^2)/a1-(d4*d6*nz^2)/a1+(d4*nx*px)/a1+(d4*ny*py)/a1+(d4*nz*pz)/a1-(cb^2*d6^2*nx^2)/(2*a1)-(cb^2*d6^2*ny^2)/(2*a1)-(cb^2*d6^2*nz^2)/(2*a1)-(cb^2*d5^2*sb^2)/a1+(d6^2*nx^2*sb^2)/(2*a1)+(d6^2*ny^2*sb^2)/(2*a1)+(d6^2*nz^2*sb^2)/(2*a1)+(cb*d5*nx*px)/a1+(cb*d5*ny*py)/a1+(cb*d5*nz*pz)/a1-(cb*d5*d6*nx^2)/a1-(cb*d5*d6*ny^2)/a1-(cb*d5*d6*nz^2)/a1-(cb*d4*d5*sb^2)/a1+(cb^2*d6*nx*px)/a1+(cb^2*d6*ny*py)/a1+(cb^2*d6*nz*pz)/a1-(d6*nx*px*sb^2)/a1-(d6*ny*py*sb^2)/a1-(d6*nz*pz*sb^2)/a1;
k51_2=pz*sb^2+d4*nz-cb^2*pz+cb^2*d6*nz-d6*nz*sb^2+cb*d5*nz-(a3*cb^2*d4)/a1-(a3*cb^3*d5)/a1-(a3*d6*nx^2)/a1-(a3*d6*ny^2)/a1-(a3*d6*nz^2)/a1+(a3*d4*sb^2)/a1+(a3*nx*px)/a1+(a3*ny*py)/a1+(a3*nz*pz)/a1-(a3*cb*d5*sb^2)/a1;
k51_3=(a2*nx*px)/a1+(a2*ny*py)/a1+(a2*nz*pz)/a1-(a2*cb^2*d4)/a1-(a2*cb^3*d5)/a1-(a2*d6*nx^2)/a1-(a2*d6*ny^2)/a1-(a2*d6*nz^2)/a1+(a2*d4*sb^2)/a1-(a2*cb*d5*sb^2)/a1;
k52_1=2*pz*sb-2*d6*nz*sb-(2*a3*d4*sb)/a1-(2*a3*cb*d5*sb)/a1;
k52_2=a1*sb+(d5^2*sb^3)/a1-(a2^2*sb)/a1-(a3^2*sb)/a1+(d4^2*sb)/a1-(px^2*sb)/a1-(py^2*sb)/a1-(pz^2*sb)/a1+(cb^2*d5^2*sb)/a1-(d6^2*nx^2*sb)/a1-(d6^2*ny^2*sb)/a1-(d6^2*nz^2*sb)/a1+(2*cb*d4*d5*sb)/a1+(2*d6*nx*px*sb)/a1+(2*d6*ny*py*sb)/a1+(2*d6*nz*pz*sb)/a1;
k52_3=-(2*a2*a3*sb)/a1;
k53_1=(a1*sb^2)/2-a3*nz+(a1*cb^2)/2-(a2^2*cb^2)/(2*a1)+(a3^2*cb^2)/(2*a1)-(cb^2*d4^2)/(2*a1)-(cb^4*d5^2)/(2*a1)-(cb^2*px^2)/(2*a1)-(cb^2*py^2)/(2*a1)-(cb^2*pz^2)/(2*a1)-(a2^2*sb^2)/(2*a1)+(a3^2*sb^2)/(2*a1)-(d4^2*sb^2)/(2*a1)+(d5^2*sb^4)/(2*a1)-(px^2*sb^2)/(2*a1)-(py^2*sb^2)/(2*a1)-(pz^2*sb^2)/(2*a1)-(cb^3*d4*d5)/a1-(d4*d6*nx^2)/a1-(d4*d6*ny^2)/a1-(d4*d6*nz^2)/a1+(d4*nx*px)/a1+(d4*ny*py)/a1+(d4*nz*pz)/a1-(cb^2*d6^2*nx^2)/(2*a1)-(cb^2*d6^2*ny^2)/(2*a1)-(cb^2*d6^2*nz^2)/(2*a1)-(d6^2*nx^2*sb^2)/(2*a1)-(d6^2*ny^2*sb^2)/(2*a1)-(d6^2*nz^2*sb^2)/(2*a1)+(cb*d5*nx*px)/a1+(cb*d5*ny*py)/a1+(cb*d5*nz*pz)/a1-(cb*d5*d6*nx^2)/a1-(cb*d5*d6*ny^2)/a1-(cb*d5*d6*nz^2)/a1-(cb*d4*d5*sb^2)/a1+(cb^2*d6*nx*px)/a1+(cb^2*d6*ny*py)/a1+(cb^2*d6*nz*pz)/a1+(d6*nx*px*sb^2)/a1+(d6*ny*py*sb^2)/a1+(d6*nz*pz*sb^2)/a1;
k53_2=-pz*sb^2+d4*nz-cb^2*pz+cb^2*d6*nz+d6*nz*sb^2+cb*d5*nz-(a3*cb^2*d4)/a1-(a3*cb^3*d5)/a1-(a3*d6*nx^2)/a1-(a3*d6*ny^2)/a1-(a3*d6*nz^2)/a1-(a3*d4*sb^2)/a1+(a3*nx*px)/a1+(a3*ny*py)/a1+(a3*nz*pz)/a1-(a3*cb*d5*sb^2)/a1;
k53_3=(a2*nx*px)/a1+(a2*ny*py)/a1+(a2*nz*pz)/a1-(a2*cb^2*d4)/a1-(a2*cb^3*d5)/a1-(a2*d6*nx^2)/a1-(a2*d6*ny^2)/a1-(a2*d6*nz^2)/a1-(a2*d4*sb^2)/a1-(a2*cb*d5*sb^2)/a1;
k54_1=4*cb*pz*sb-2*d5*nz*sb-(2*a3*d5*sb^3)/a1-4*cb*d6*nz*sb-(4*a3*cb*d4*sb)/a1-(2*a3*cb^2*d5*sb)/a1;
k54_2=2*a1*cb*sb-(2*a2^2*cb*sb)/a1-(2*a3^2*cb*sb)/a1+(2*cb*d4^2*sb)/a1+(2*d4*d5*sb^3)/a1-(2*cb*px^2*sb)/a1-(2*cb*py^2*sb)/a1-(2*cb*pz^2*sb)/a1+(2*d5*nx*px*sb)/a1+(2*d5*ny*py*sb)/a1+(2*d5*nz*pz*sb)/a1+(2*cb^2*d4*d5*sb)/a1-(2*d5*d6*nx^2*sb)/a1-(2*d5*d6*ny^2*sb)/a1-(2*d5*d6*nz^2*sb)/a1-(2*cb*d6^2*nx^2*sb)/a1-(2*cb*d6^2*ny^2*sb)/a1-(2*cb*d6^2*nz^2*sb)/a1+(4*cb*d6*nx*px*sb)/a1+(4*cb*d6*ny*py*sb)/a1+(4*cb*d6*nz*pz*sb)/a1;
k54_3=-(4*a2*a3*cb*sb)/a1;
k56_1=(2*a3*d5*sb^3)/a1 - 2*d5*nz*sb + (2*a3*cb^2*d5*sb)/a1;
k56_2=- (2*d4*d5*sb^3)/a1 - (2*cb*d5^2*sb^3)/a1 - (2*cb^3*d5^2*sb)/a1 + (2*d5*nx*px*sb)/a1 + (2*d5*ny*py*sb)/a1 + (2*d5*nz*pz*sb)/a1 - (2*cb^2*d4*d5*sb)/a1 - (2*d5*d6*nx^2*sb)/a1 - (2*d5*d6*ny^2*sb)/a1 - (2*d5*d6*nz^2*sb)/a1;
k57_1=-(a1*sb^2)/2-a3*nz+(a1*cb^2)/2-(a2^2*cb^2)/(2*a1)+(a3^2*cb^2)/(2*a1)-(cb^2*d4^2)/(2*a1)-(cb^4*d5^2)/(2*a1)-(cb^2*px^2)/(2*a1)-(cb^2*py^2)/(2*a1)-(cb^2*pz^2)/(2*a1)+(a2^2*sb^2)/(2*a1)-(a3^2*sb^2)/(2*a1)+(d4^2*sb^2)/(2*a1)-(d5^2*sb^4)/(2*a1)+(px^2*sb^2)/(2*a1)+(py^2*sb^2)/(2*a1)+(pz^2*sb^2)/(2*a1)-(cb^3*d4*d5)/a1-(d4*d6*nx^2)/a1-(d4*d6*ny^2)/a1-(d4*d6*nz^2)/a1+(d4*nx*px)/a1+(d4*ny*py)/a1+(d4*nz*pz)/a1-(cb^2*d6^2*nx^2)/(2*a1)-(cb^2*d6^2*ny^2)/(2*a1)-(cb^2*d6^2*nz^2)/(2*a1)-(cb^2*d5^2*sb^2)/a1+(d6^2*nx^2*sb^2)/(2*a1)+(d6^2*ny^2*sb^2)/(2*a1)+(d6^2*nz^2*sb^2)/(2*a1)+(cb*d5*nx*px)/a1+(cb*d5*ny*py)/a1+(cb*d5*nz*pz)/a1-(cb*d5*d6*nx^2)/a1-(cb*d5*d6*ny^2)/a1-(cb*d5*d6*nz^2)/a1-(cb*d4*d5*sb^2)/a1+(cb^2*d6*nx*px)/a1+(cb^2*d6*ny*py)/a1+(cb^2*d6*nz*pz)/a1-(d6*nx*px*sb^2)/a1-(d6*ny*py*sb^2)/a1-(d6*nz*pz*sb^2)/a1;
k57_2=pz*sb^2+d4*nz-cb^2*pz+cb^2*d6*nz-d6*nz*sb^2+cb*d5*nz-(a3*cb^2*d4)/a1-(a3*cb^3*d5)/a1-(a3*d6*nx^2)/a1-(a3*d6*ny^2)/a1-(a3*d6*nz^2)/a1+(a3*d4*sb^2)/a1+(a3*nx*px)/a1+(a3*ny*py)/a1+(a3*nz*pz)/a1-(a3*cb*d5*sb^2)/a1;
k57_3=(a2*nx*px)/a1+(a2*ny*py)/a1+(a2*nz*pz)/a1-(a2*cb^2*d4)/a1-(a2*cb^3*d5)/a1-(a2*d6*nx^2)/a1-(a2*d6*ny^2)/a1-(a2*d6*nz^2)/a1+(a2*d4*sb^2)/a1-(a2*cb*d5*sb^2)/a1;
k58_1=2*d6*nz*sb-2*pz*sb+(2*a3*d4*sb)/a1+(2*a3*cb*d5*sb)/a1;
k58_2=-a1*sb-(d5^2*sb^3)/a1+(a2^2*sb)/a1+(a3^2*sb)/a1-(d4^2*sb)/a1+(px^2*sb)/a1+(py^2*sb)/a1+(pz^2*sb)/a1-(cb^2*d5^2*sb)/a1+(d6^2*nx^2*sb)/a1+(d6^2*ny^2*sb)/a1+(d6^2*nz^2*sb)/a1-(2*cb*d4*d5*sb)/a1-(2*d6*nx*px*sb)/a1-(2*d6*ny*py*sb)/a1-(2*d6*nz*pz*sb)/a1;
k58_3=(2*a2*a3*sb)/a1;
k59_1=(a1*sb^2)/2-a3*nz+(a1*cb^2)/2-(a2^2*cb^2)/(2*a1)+(a3^2*cb^2)/(2*a1)-(cb^2*d4^2)/(2*a1)-(cb^4*d5^2)/(2*a1)-(cb^2*px^2)/(2*a1)-(cb^2*py^2)/(2*a1)-(cb^2*pz^2)/(2*a1)-(a2^2*sb^2)/(2*a1)+(a3^2*sb^2)/(2*a1)-(d4^2*sb^2)/(2*a1)+(d5^2*sb^4)/(2*a1)-(px^2*sb^2)/(2*a1)-(py^2*sb^2)/(2*a1)-(pz^2*sb^2)/(2*a1)-(cb^3*d4*d5)/a1-(d4*d6*nx^2)/a1-(d4*d6*ny^2)/a1-(d4*d6*nz^2)/a1+(d4*nx*px)/a1+(d4*ny*py)/a1+(d4*nz*pz)/a1-(cb^2*d6^2*nx^2)/(2*a1)-(cb^2*d6^2*ny^2)/(2*a1)-(cb^2*d6^2*nz^2)/(2*a1)-(d6^2*nx^2*sb^2)/(2*a1)-(d6^2*ny^2*sb^2)/(2*a1)-(d6^2*nz^2*sb^2)/(2*a1)+(cb*d5*nx*px)/a1+(cb*d5*ny*py)/a1+(cb*d5*nz*pz)/a1-(cb*d5*d6*nx^2)/a1-(cb*d5*d6*ny^2)/a1-(cb*d5*d6*nz^2)/a1-(cb*d4*d5*sb^2)/a1+(cb^2*d6*nx*px)/a1+(cb^2*d6*ny*py)/a1+(cb^2*d6*nz*pz)/a1+(d6*nx*px*sb^2)/a1+(d6*ny*py*sb^2)/a1+(d6*nz*pz*sb^2)/a1;
k59_2=-pz*sb^2+d4*nz-cb^2*pz+cb^2*d6*nz+d6*nz*sb^2+cb*d5*nz-(a3*cb^2*d4)/a1-(a3*cb^3*d5)/a1-(a3*d6*nx^2)/a1-(a3*d6*ny^2)/a1-(a3*d6*nz^2)/a1-(a3*d4*sb^2)/a1+(a3*nx*px)/a1+(a3*ny*py)/a1+(a3*nz*pz)/a1-(a3*cb*d5*sb^2)/a1;
k59_3=(a2*nx*px)/a1+(a2*ny*py)/a1+(a2*nz*pz)/a1-(a2*cb^2*d4)/a1-(a2*cb^3*d5)/a1-(a2*d6*nx^2)/a1-(a2*d6*ny^2)/a1-(a2*d6*nz^2)/a1-(a2*d4*sb^2)/a1-(a2*cb*d5*sb^2)/a1;
k61_1=cb^2*pz-pz*sb^2-d4*nz-cb*d5*nz-cb^2*d6*nz+d6*nz*sb^2+(a3*cb^2*d4)/a1+(a3*cb^3*d5)/a1+(a3*d6*nx^2)/a1+(a3*d6*ny^2)/a1+(a3*d6*nz^2)/a1-(a3*d4*sb^2)/a1-(a3*nx*px)/a1-(a3*ny*py)/a1-(a3*nz*pz)/a1+(a3*cb*d5*sb^2)/a1;
k61_2=-a3*nz+(a1*cb^2)/2-(a1*sb^2)/2+(a2^2*cb^2)/(2*a1)+(a3^2*cb^2)/(2*a1)-(cb^2*d4^2)/(2*a1)-(cb^4*d5^2)/(2*a1)-(cb^2*px^2)/(2*a1)-(cb^2*py^2)/(2*a1)-(cb^2*pz^2)/(2*a1)-(a2^2*sb^2)/(2*a1)-(a3^2*sb^2)/(2*a1)+(d4^2*sb^2)/(2*a1)-(d5^2*sb^2)/(2*a1)+(px^2*sb^2)/(2*a1)+(py^2*sb^2)/(2*a1)+(pz^2*sb^2)/(2*a1)-(cb^3*d4*d5)/a1-(d4*d6*nx^2)/a1-(d4*d6*ny^2)/a1-(d4*d6*nz^2)/a1+(d4*nx*px)/a1+(d4*ny*py)/a1+(d4*nz*pz)/a1-(cb^2*d6^2*nx^2)/(2*a1)-(cb^2*d6^2*ny^2)/(2*a1)-(cb^2*d6^2*nz^2)/(2*a1)-(cb^2*d5^2*sb^2)/(2*a1)+(d6^2*nx^2*sb^2)/(2*a1)+(d6^2*ny^2*sb^2)/(2*a1)+(d6^2*nz^2*sb^2)/(2*a1)+(cb*d5*nx*px)/a1+(cb*d5*ny*py)/a1+(cb*d5*nz*pz)/a1-(cb*d5*d6*nx^2)/a1-(cb*d5*d6*ny^2)/a1-(cb*d5*d6*nz^2)/a1-(cb*d4*d5*sb^2)/a1+(cb^2*d6*nx*px)/a1+(cb^2*d6*ny*py)/a1+(cb^2*d6*nz*pz)/a1-(d6*nx*px*sb^2)/a1-(d6*ny*py*sb^2)/a1-(d6*nz*pz*sb^2)/a1;
k61_3=-a2*nz+(a2*a3*cb^2)/a1-(a2*a3*sb^2)/a1;
k62_1=-a1*sb-(a2^2*sb)/a1+(a3^2*sb)/a1-(d4^2*sb)/a1-(d5^2*sb)/a1+(px^2*sb)/a1+(py^2*sb)/a1+(pz^2*sb)/a1+(d6^2*nx^2*sb)/a1+(d6^2*ny^2*sb)/a1+(d6^2*nz^2*sb)/a1-(2*cb*d4*d5*sb)/a1-(2*d6*nx*px*sb)/a1-(2*d6*ny*py*sb)/a1-(2*d6*nz*pz*sb)/a1;
k62_2=2*pz*sb-2*d6*nz*sb-(2*a3*d4*sb)/a1-(2*a3*cb*d5*sb)/a1;
k62_3=-(2*a2*d4*sb)/a1-(2*a2*cb*d5*sb)/a1;
k63_1=cb^2*pz+pz*sb^2-d4*nz-cb*d5*nz-cb^2*d6*nz-d6*nz*sb^2+(a3*cb^2*d4)/a1+(a3*cb^3*d5)/a1+(a3*d6*nx^2)/a1+(a3*d6*ny^2)/a1+(a3*d6*nz^2)/a1+(a3*d4*sb^2)/a1-(a3*nx*px)/a1-(a3*ny*py)/a1-(a3*nz*pz)/a1+(a3*cb*d5*sb^2)/a1;
k63_2=-a3*nz+(a1*cb^2)/2+(a1*sb^2)/2+(a2^2*cb^2)/(2*a1)+(a3^2*cb^2)/(2*a1)-(cb^2*d4^2)/(2*a1)-(cb^4*d5^2)/(2*a1)-(cb^2*px^2)/(2*a1)-(cb^2*py^2)/(2*a1)-(cb^2*pz^2)/(2*a1)+(a2^2*sb^2)/(2*a1)+(a3^2*sb^2)/(2*a1)-(d4^2*sb^2)/(2*a1)+(d5^2*sb^2)/(2*a1)-(px^2*sb^2)/(2*a1)-(py^2*sb^2)/(2*a1)-(pz^2*sb^2)/(2*a1)-(cb^3*d4*d5)/a1-(d4*d6*nx^2)/a1-(d4*d6*ny^2)/a1-(d4*d6*nz^2)/a1+(d4*nx*px)/a1+(d4*ny*py)/a1+(d4*nz*pz)/a1-(cb^2*d6^2*nx^2)/(2*a1)-(cb^2*d6^2*ny^2)/(2*a1)-(cb^2*d6^2*nz^2)/(2*a1)-(cb^2*d5^2*sb^2)/(2*a1)-(d6^2*nx^2*sb^2)/(2*a1)-(d6^2*ny^2*sb^2)/(2*a1)-(d6^2*nz^2*sb^2)/(2*a1)+(cb*d5*nx*px)/a1+(cb*d5*ny*py)/a1+(cb*d5*nz*pz)/a1-(cb*d5*d6*nx^2)/a1-(cb*d5*d6*ny^2)/a1-(cb*d5*d6*nz^2)/a1-(cb*d4*d5*sb^2)/a1+(cb^2*d6*nx*px)/a1+(cb^2*d6*ny*py)/a1+(cb^2*d6*nz*pz)/a1+(d6*nx*px*sb^2)/a1+(d6*ny*py*sb^2)/a1+(d6*nz*pz*sb^2)/a1;
k63_3=-a2*nz+(a2*a3*cb^2)/a1+(a2*a3*sb^2)/a1;
k64_1=-2*a1*cb*sb-(2*a2^2*cb*sb)/a1+(2*a3^2*cb*sb)/a1-(2*cb*d4^2*sb)/a1+(2*cb*px^2*sb)/a1+(2*cb*py^2*sb)/a1+(2*cb*pz^2*sb)/a1-(2*d4*d5*sb)/a1-(2*d5*nx*px*sb)/a1-(2*d5*ny*py*sb)/a1-(2*d5*nz*pz*sb)/a1+(2*d5*d6*nx^2*sb)/a1+(2*d5*d6*ny^2*sb)/a1+(2*d5*d6*nz^2*sb)/a1+(2*cb*d6^2*nx^2*sb)/a1+(2*cb*d6^2*ny^2*sb)/a1+(2*cb*d6^2*nz^2*sb)/a1-(4*cb*d6*nx*px*sb)/a1-(4*cb*d6*ny*py*sb)/a1-(4*cb*d6*nz*pz*sb)/a1;
k64_2=4*cb*pz*sb-2*d5*nz*sb-4*cb*d6*nz*sb-(2*a3*d5*sb)/a1-(4*a3*cb*d4*sb)/a1;
k64_3=-(2*a2*d5*sb)/a1-(4*a2*cb*d4*sb)/a1;
k66_1=(2*cb*d5^2*sb)/a1+(2*d4*d5*sb)/a1-(2*d5*nx*px*sb)/a1-(2*d5*ny*py*sb)/a1-(2*d5*nz*pz*sb)/a1+(2*d5*d6*nx^2*sb)/a1+(2*d5*d6*ny^2*sb)/a1+(2*d5*d6*nz^2*sb)/a1;
k66_2=-2*d5*nz*sb+(2*a3*d5*sb)/a1;
k66_3=(2*a2*d5*sb)/a1;
k67_1=cb^2*pz-pz*sb^2-d4*nz-cb*d5*nz-cb^2*d6*nz+d6*nz*sb^2+(a3*cb^2*d4)/a1+(a3*cb^3*d5)/a1+(a3*d6*nx^2)/a1+(a3*d6*ny^2)/a1+(a3*d6*nz^2)/a1-(a3*d4*sb^2)/a1-(a3*nx*px)/a1-(a3*ny*py)/a1-(a3*nz*pz)/a1+(a3*cb*d5*sb^2)/a1;
k67_2=-a3*nz+(a1*cb^2)/2-(a1*sb^2)/2+(a2^2*cb^2)/(2*a1)+(a3^2*cb^2)/(2*a1)-(cb^2*d4^2)/(2*a1)-(cb^4*d5^2)/(2*a1)-(cb^2*px^2)/(2*a1)-(cb^2*py^2)/(2*a1)-(cb^2*pz^2)/(2*a1)-(a2^2*sb^2)/(2*a1)-(a3^2*sb^2)/(2*a1)+(d4^2*sb^2)/(2*a1)-(d5^2*sb^2)/(2*a1)+(px^2*sb^2)/(2*a1)+(py^2*sb^2)/(2*a1)+(pz^2*sb^2)/(2*a1)-(cb^3*d4*d5)/a1-(d4*d6*nx^2)/a1-(d4*d6*ny^2)/a1-(d4*d6*nz^2)/a1+(d4*nx*px)/a1+(d4*ny*py)/a1+(d4*nz*pz)/a1-(cb^2*d6^2*nx^2)/(2*a1)-(cb^2*d6^2*ny^2)/(2*a1)-(cb^2*d6^2*nz^2)/(2*a1)-(cb^2*d5^2*sb^2)/(2*a1)+(d6^2*nx^2*sb^2)/(2*a1)+(d6^2*ny^2*sb^2)/(2*a1)+(d6^2*nz^2*sb^2)/(2*a1)+(cb*d5*nx*px)/a1+(cb*d5*ny*py)/a1+(cb*d5*nz*pz)/a1-(cb*d5*d6*nx^2)/a1-(cb*d5*d6*ny^2)/a1-(cb*d5*d6*nz^2)/a1-(cb*d4*d5*sb^2)/a1+(cb^2*d6*nx*px)/a1+(cb^2*d6*ny*py)/a1+(cb^2*d6*nz*pz)/a1-(d6*nx*px*sb^2)/a1-(d6*ny*py*sb^2)/a1-(d6*nz*pz*sb^2)/a1;
k67_3=-a2*nz+(a2*a3*cb^2)/a1-(a2*a3*sb^2)/a1;
k68_1=a1*sb+(a2^2*sb)/a1-(a3^2*sb)/a1+(d4^2*sb)/a1+(d5^2*sb)/a1-(px^2*sb)/a1-(py^2*sb)/a1-(pz^2*sb)/a1-(d6^2*nx^2*sb)/a1-(d6^2*ny^2*sb)/a1-(d6^2*nz^2*sb)/a1+(2*cb*d4*d5*sb)/a1+(2*d6*nx*px*sb)/a1+(2*d6*ny*py*sb)/a1+(2*d6*nz*pz*sb)/a1;
k68_2=-2*pz*sb+2*d6*nz*sb+(2*a3*d4*sb)/a1+(2*a3*cb*d5*sb)/a1;
k68_3=(2*a2*d4*sb)/a1+(2*a2*cb*d5*sb)/a1;
k69_1=cb^2*pz+pz*sb^2-d4*nz-cb*d5*nz-cb^2*d6*nz-d6*nz*sb^2+(a3*cb^2*d4)/a1+(a3*cb^3*d5)/a1+(a3*d6*nx^2)/a1+(a3*d6*ny^2)/a1+(a3*d6*nz^2)/a1+(a3*d4*sb^2)/a1-(a3*nx*px)/a1-(a3*ny*py)/a1-(a3*nz*pz)/a1+(a3*cb*d5*sb^2)/a1;
k69_2=-a3*nz+(a1*cb^2)/2+(a1*sb^2)/2+(a2^2*cb^2)/(2*a1)+(a3^2*cb^2)/(2*a1)-(cb^2*d4^2)/(2*a1)-(cb^4*d5^2)/(2*a1)-(cb^2*px^2)/(2*a1)-(cb^2*py^2)/(2*a1)-(cb^2*pz^2)/(2*a1)+(a2^2*sb^2)/(2*a1)+(a3^2*sb^2)/(2*a1)-(d4^2*sb^2)/(2*a1)+(d5^2*sb^2)/(2*a1)-(px^2*sb^2)/(2*a1)-(py^2*sb^2)/(2*a1)-(pz^2*sb^2)/(2*a1)-(cb^3*d4*d5)/a1-(d4*d6*nx^2)/a1-(d4*d6*ny^2)/a1-(d4*d6*nz^2)/a1+(d4*nx*px)/a1+(d4*ny*py)/a1+(d4*nz*pz)/a1-(cb^2*d6^2*nx^2)/(2*a1)-(cb^2*d6^2*ny^2)/(2*a1)-(cb^2*d6^2*nz^2)/(2*a1)-(cb^2*d5^2*sb^2)/(2*a1)-(d6^2*nx^2*sb^2)/(2*a1)-(d6^2*ny^2*sb^2)/(2*a1)-(d6^2*nz^2*sb^2)/(2*a1)+(cb*d5*nx*px)/a1+(cb*d5*ny*py)/a1+(cb*d5*nz*pz)/a1-(cb*d5*d6*nx^2)/a1-(cb*d5*d6*ny^2)/a1-(cb*d5*d6*nz^2)/a1-(cb*d4*d5*sb^2)/a1+(cb^2*d6*nx*px)/a1+(cb^2*d6*ny*py)/a1+(cb^2*d6*nz*pz)/a1+(d6*nx*px*sb^2)/a1+(d6*ny*py*sb^2)/a1+(d6*nz*pz*sb^2)/a1;
k69_3=-a2*nz+(a2*a3*cb^2)/a1+(a2*a3*sb^2)/a1;
%Constructing M matrix
A=[k11_3-k11_2,0,k13_3-k13_2,k14_3-k14_2,k15_3,k16_3-k16_2,k17_3-k17_2,0,k19_3-k19_2,0,0,0;k21_3,k22_3-k22_2,k23_3,k24_3-k24_2,k25_3,0,k27_3,k28_3-k28_2,k29_3,0,0,0;k31_3-k31_2,k32_3,k33_3-k33_2,k34_3,k35_3,k36_3,k37_3-k37_2,k38_3,k39_3-k39_2,0,0,0;k41_3-k41_2,k42_3-k42_2,k43_3,0,k45_3-k45_2,0,k47_3-k47_2,k48_3-k48_2,k49_3,0,0,0;k51_3-k51_2,k52_3-k52_2,k53_3-k53_2,k54_3-k54_2,0,-k56_2,k57_3-k57_2,k58_3-k58_2,k59_3-k59_2,0,0,0;k61_3-k61_2,k62_3-k62_2,k63_3-k63_2,k64_3-k64_2,0,k66_3-k66_2,k67_3-k67_2,k68_3-k68_2,k69_3-k69_2,0,0,0;0,0,0,k11_3-k11_2,0,k13_3-k13_2,k14_3-k14_2,k15_3,k16_3-k16_2,k17_3-k17_2,0,k19_3-k19_2;0,0,0,k21_3,k22_3-k22_2,k23_3,k24_3-k24_2,k25_3,0,k27_3,k28_3-k28_2,k29_3;0,0,0,k31_3-k31_2,k32_3,k33_3-k33_2,k34_3,k35_3,k36_3,k37_3-k37_2,k38_3,k39_3-k39_2;0,0,0,k41_3-k41_2,k42_3-k42_2,k43_3,0,k45_3-k45_2,0,k47_3-k47_2,k48_3-k48_2,k49_3;0,0,0,k51_3-k51_2,k52_3-k52_2,k53_3-k53_2,k54_3-k54_2,0,-k56_2,k57_3-k57_2,k58_3-k58_2,k59_3-k59_2;0,0,0,k61_3-k61_2,k62_3-k62_2,k63_3-k63_2,k64_3-k64_2,0,k66_3-k66_2,k67_3-k67_2,k68_3-k68_2,k69_3-k69_2];
B=[2*k11_1,0,2*k13_1,0,0,0,2*k17_1,0,2*k19_1,0,0,0;2*k21_1,0,2*k23_1,0,0,0,2*k27_1,0,2*k29_1,0,0,0;0,2*k32_1,0,2*k34_1,0,0,0,2*k38_1,0,0,0,0;2*k41_1,0,2*k43_1,0,2*k45_1,0,2*k47_1,0,2*k49_1,0,0,0;2*k51_1,2*k52_1,2*k53_1,2*k54_1,0,2*k56_1,2*k57_1,2*k58_1,2*k59_1,0,0,0;2*k61_1,2*k62_1,2*k63_1,2*k64_1,0,2*k66_1,2*k67_1,2*k68_1,2*k69_1,0,0,0;0,0,0,2*k11_1,0,2*k13_1,0,0,0,2*k17_1,0,2*k19_1;0,0,0,2*k21_1,0,2*k23_1,0,0,0,2*k27_1,0,2*k29_1;0,0,0,0,2*k32_1,0,2*k34_1,0,0,0,2*k38_1,0;0,0,0,2*k41_1,0,2*k43_1,0,2*k45_1,0,2*k47_1,0,2*k49_1;0,0,0,2*k51_1,2*k52_1,2*k53_1,2*k54_1,0,2*k56_1,2*k57_1,2*k58_1,2*k59_1;0,0,0,2*k61_1,2*k62_1,2*k63_1,2*k64_1,0,2*k66_1,2*k67_1,2*k68_1,2*k69_1];
C=[k11_3+k11_2,0,k13_3+k13_2,k14_3+k14_2,k15_3,k16_3+k16_2,k17_3+k17_2,0,k19_3+k19_2,0,0,0;k21_3,k22_3+k22_2,k23_3,k24_3+k24_2,k25_3,0,k27_3,k28_3+k28_2,k29_3,0,0,0;k31_3+k31_2,k32_3,k33_3+k33_2,k34_3,k35_3,k36_3,k37_3+k37_2,k38_3,k39_3+k39_2,0,0,0;k41_3+k41_2,k42_3+k42_2,k43_3,0,k45_3+k45_2,0,k47_3+k47_2,k48_3+k48_2,k49_3,0,0,0;k51_3+k51_2,k52_3+k52_2,k53_3+k53_2,k54_3+k54_2,0,k56_2,k57_3+k57_2,k58_3+k58_2,k59_3+k59_2,0,0,0;k61_3+k61_2,k62_3+k62_2,k63_3+k63_2,k64_3+k64_2,0,k66_3+k66_2,k67_3+k67_2,k68_3+k68_2,k69_3+k69_2,0,0,0;0,0,0,k11_3+k11_2,0,k13_3+k13_2,k14_3+k14_2,k15_3,k16_3+k16_2,k17_3+k17_2,0,k19_3+k19_2;0,0,0,k21_3,k22_3+k22_2,k23_3,k24_3+k24_2,k25_3,0,k27_3,k28_3+k28_2,k29_3;0,0,0,k31_3+k31_2,k32_3,k33_3+k33_2,k34_3,k35_3,k36_3,k37_3+k37_2,k38_3,k39_3+k39_2;0,0,0,k41_3+k41_2,k42_3+k42_2,k43_3,0,k45_3+k45_2,0,k47_3+k47_2,k48_3+k48_2,k49_3;0,0,0,k51_3+k51_2,k52_3+k52_2,k53_3+k53_2,k54_3+k54_2,0,k56_2,k57_3+k57_2,k58_3+k58_2,k59_3+k59_2;0,0,0,k61_3+k61_2,k62_3+k62_2,k63_3+k63_2,k64_3+k64_2,0,k66_3+k66_2,k67_3+k67_2,k68_3+k68_2,k69_3+k69_2];
O=zeros(12);
I=eye(12);
M=[O,I;-inv(A)*C,-inv(A)*B];
%Eigenvalues and eigenvectors of M matrices,print eigenvalues 
[V,D]=eig(M);
D=diag(D);
eigenvalues=[];
for i=1:length(D)
    if isreal(D(i))==1
       eigenvalues(length(eigenvalues)+1)=D(i);
    end
end
vpa(eigenvalues,8)
%Joint Angle q1~q6
for i=1:length(D)
     if isreal(D(i))==1
         %q3
         x3=D(i);
         q3= 2*atand(D(i));
         v=V(:,i);
         %q4,q5
         if abs(x3)<=1
            v=v(1:12);
            if max(abs(v))==abs(v(1))
                q4=2*atand(v(1)/v(4));
                q5=2*atand(v(1)/v(2));
            elseif max(abs(v))==abs(v(3))
                q4=2*atand(v(3)/v(6));
                q5=2*atand(v(2)/v(3));
            elseif max(abs(v))==abs(v(10))
                q4=2*atand(v(7)/v(10));
                q5=2*atand(v(10)/v(11));
            elseif max(abs(v))==abs(v(12))
                q4=2*atand(v(9)/v(12));
                q5=2*atand(v(11)/v(12));
            end
         else
            v=v(13:24);
            if max(abs(v))==abs(v(1))
                q4=2*atand(v(1)/v(4));
                q5=2*atand(v(1)/v(2));
            elseif max(abs(v))==abs(v(3))
                q4=2*atand(v(3)/v(6));
                q5=2*atand(v(2)/v(3));
            elseif max(abs(v))==abs(v(10))
                q4=2*atand(v(7)/v(10));
                q5=2*atand(v(10)/v(11));
            elseif max(abs(v))==abs(v(12))
                q4=2*atand(v(9)/v(12));
                q5=2*atand(v(11)/v(12));
            end
         end
         %q1
         c4=cosd(q4);
         s4=sind(q4);
         c5=cosd(q5);
         s5=sind(q5);
         s1 =((py - d6*ny)*(c4*cb*sb + s4*s5*sb - c4*c5*cb*sb) - c4*d5*ny*sb)/(ny*px - nx*py);
         c1 =((px - d6*nx)*(c4*cb*sb + s4*s5*sb - c4*c5*cb*sb) - c4*d5*nx*sb)/(ny*px - nx*py);
         q1=asind(s1);
         re1=[];
         re1(length(re1)+1)=q1;
            if s1>0&c1<0
                re1(length(re1)+1)=180-q1;
            else
                re1(length(re1)+1)=-180-q1;
            end
         re6=[];
         for i=1:length(re1)
            q1=re1(i);
            %q6
            c1=cosd(q1);
            s1=sind(q1);            
            l = (d5*cb*(ny*c1 - nx*s1)+d6*(ny*c1 - nx*s1) - py*c1 + px*s1)/(d5*sb);
            n = my*c1 - mx*s1;
            m = ly*c1 - lx*s1;
            x61=(m+sqrt(m*m+n*n-l*l))/(l+n);
            x62=(m-sqrt(m*m+n*n-l*l))/(l+n); 
            q61=2*atand(x61);
            q62=2*atand(x62);
            re6(length(re6)+1)=q61;
            re6(length(re6)+1)=q62;
            for i=1:length(re6)
                q6=re6(i);
                if isreal(q6)==1
                    %q2
                    c3=cosd(q3);
                    s3=sind(q3);
                    h1= -a1- d6*nx*c1- d6*ny*s1+ px*c1 + py*s1;
                    h2= pz - d6*nz;
                    g1=c3*(d5*sb*s4 + a3)+s3*(d4 + d5*cb)+a2;
                    g2=c3*(d4 + d5*cb)-s3*(d5*sb*s4 + a3);
                    s2=(h1*g2+h2*g1)/(h1^2+h2^2);
                    c2=(g1*h1-g2*h2)/(h1^2+h2^2);
                    q21=asind(s2);
                    if s2>0&c2<0
                        q22=180-q21;
                    else
                        q22=-180-q21;
                    end
                    ret=vpa([q1,q21,q3,q4,q5,q6],8)
                    ret=vpa([q1,q22,q3,q4,q5,q6],8)
                end
            end
         end
     end
end
