clear;
close all;
% Robotic constants defination
sample_time = 0.001;
g = 9.8;
Zc = 0.2;

% discreate space state parameters
A = [1 sample_time sample_time^2/2; 0 1 sample_time;0 0 1];
B = [sample_time^3/6;sample_time^2/2;sample_time];
C = [1 0 -Zc/g];

% extend space state parameters
A_extend = [1 C*A;zeros(length(A),1) A];
B_extend = [C*B;B];
C_extend = [1 0 0 0];

Q_extend = 1;
Q_Square = C_extend'*Q_extend*C_extend;
R_extend  = 1e-4;

[K_extend,P_extend,e] = dlqr(A_extend,B_extend,Q_Square,R_extend);

Prv_Time = 1.4;
N = 1400; % fulture number

% parameters generate
G_extend = zeros(N,1);

G_extend(1) = -K_extend(1);
Ac_extend = A_extend - B_extend*K_extend;
X_extend = -Ac_extend'*P_extend*[1;0;0;0];
tmp = (R_extend + B_extend'*P_extend*B_extend)^-1*B_extend';
for i=2:N
    G_extend(i) = tmp*X_extend;
    X_extend = Ac_extend'*X_extend;
end

fid = fopen('preview_control_parameters_generate.txt','w');
fprintf(fid,"#pragma once \r\n");
fprintf(fid,"#define PREVIEW_CONTROL_SAMPLE_TIME %f\r\n",sample_time);
fprintf(fid,"#define PREVIEW_CONTROL_ACC_G %f\r\n",g);

[m,n] = size(A);
fprintf(fid,"#define PREVIEW_CONTROL_A_ROW %d \r\n",m);
fprintf(fid,"#define PREVIEW_CONTROL_A_COL %d \r\n",n);
fprintf(fid,"#define PREVIEW_CONTROL_A_DATA ");
for i=1:m
    fprintf(fid,"{");
    for j=1:n-1
        fprintf(fid,"%.12f,",A(i,j));
    end
    fprintf(fid,"%.12f",A(i,n));
    fprintf(fid,"}");
    if(i~=m)
        fprintf(fid,",");
    end
end
fprintf(fid,"\r\n");

[m,n] = size(B);
fprintf(fid,"#define PREVIEW_CONTROL_B_ROW %d \r\n",m);
fprintf(fid,"#define PREVIEW_CONTROL_B_COL %d \r\n",n);
fprintf(fid,"#define PREVIEW_CONTROL_B_DATA ");
for i=1:m
    fprintf(fid,"{");
    for j=1:n-1
        fprintf(fid,"%.12f,",B(i,j));
    end
    fprintf(fid,"%.12f",B(i,n));
    fprintf(fid,"}");
    if(i~=m)
        fprintf(fid,",");
    end
end
fprintf(fid,"\r\n");

[m,n] = size(C);
fprintf(fid,"#define PREVIEW_CONTROL_C_ROW %d \r\n",m);
fprintf(fid,"#define PREVIEW_CONTROL_C_COL %d \r\n",n);
fprintf(fid,"#define PREVIEW_CONTROL_C_DATA ");
for i=1:m
    fprintf(fid,"{");
    for j=1:n-1
        fprintf(fid,"%.12f,",C(i,j));
    end
    fprintf(fid,"%.12f",C(i,n));
    fprintf(fid,"}");
    if(i~=m)
        fprintf(fid,",");
    end
end
fprintf(fid,"\r\n");

m = N;
n = 1;
fprintf(fid,"#define PREVIEW_CONTROL_G_ROW %d \r\n",m);
fprintf(fid,"#define PREVIEW_CONTROL_G_COL %d \r\n",n);
fprintf(fid,"#define PREVIEW_CONTROL_G_DATA ");
for i=1:m
    fprintf(fid,"{");
    for j=1:n-1
        fprintf(fid,"%.12f,",G_extend(i,j));
    end
    fprintf(fid,"%.12f",G_extend(i,n));
    fprintf(fid,"}");
    if(i~=m)
        fprintf(fid,",");
    end
end
fprintf(fid,"\r\n");

fprintf(fid,"#define PREVIEW_CONTROL_PREVIEW_NUMBER %d \r\n",N);

m = 4;
n = 1;
fprintf(fid,"#define PREVIEW_CONTROL_K_ROW %d \r\n",m);
fprintf(fid,"#define PREVIEW_CONTROL_K_COL %d \r\n",n);
fprintf(fid,"#define PREVIEW_CONTROL_K_DATA ");
for i=1:m
    fprintf(fid,"{");
    for j=1:n-1
        fprintf(fid,"%.12f,",K_extend(j,i));
    end
    fprintf(fid,"%.12f",K_extend(n,i));
    fprintf(fid,"}");
    if(i~=m)
        fprintf(fid,",");
    end
end
fprintf(fid,"\r\n");
fclose(fid);