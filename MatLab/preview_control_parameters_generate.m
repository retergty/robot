clear;
close all;
% Robotic constants defination
sample_time = 0.001;
g = 9.8;
Zc = 0.23;

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
G_extend = zeros(1,N);

G_extend(1) = -K_extend(1);
Ac_extend = A_extend - B_extend*K_extend;
X_extend = -Ac_extend'*P_extend*[1;0;0;0];
tmp = (R_extend + B_extend'*P_extend*B_extend)^-1*B_extend';
for i=2:N
    G_extend(i) = tmp*X_extend;
    X_extend = Ac_extend'*X_extend;
end

fid = fopen('prvctl_parm.txt','w');

fprintf(fid,"sample_time %f\r\n",sample_time);

[m,n] = size(A);
fprintf(fid,"A %d %d ",m,n);
for i=1:m
    for j=1:n
        fprintf(fid,"%.12f ",A(i,j));
    end
end
fprintf(fid,"\r\n");

[m,n] = size(B);
fprintf(fid,"B %d %d ",m,n);
for i=1:m
    for j=1:n
        fprintf(fid,"%.12f ",B(i,j));
    end
end
fprintf(fid,"\r\n");

[m,n] = size(C);
fprintf(fid,"C %d %d ",m,n);
for i=1:m
    for j=1:n
        fprintf(fid,"%.12f ",C(i,j));
    end
end
fprintf(fid,"\r\n");

fprintf(fid,"G %d 1 ",N);
for i=1:N
    fprintf(fid,"%.12f ",G_extend(i));
end
fprintf(fid,"\r\n");

fprintf(fid,"K 1 4 ");
for i=1:4
    fprintf(fid,"%.12f ",K_extend(i));
end

fclose(fid);