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
G_extend = zeros(1,N);

G_extend(1) = -K_extend(1);
Ac_extend = A_extend - B_extend*K_extend;
X_extend = -Ac_extend'*P_extend*[1;0;0;0];
tmp = (R_extend + B_extend'*P_extend*B_extend)^-1*B_extend';
for i=2:N
    G_extend(i) = tmp*X_extend;
    X_extend = Ac_extend'*X_extend;
end
figure(1);
G_time = 0:sample_time:100;
G_time = G_time(1:N);
plot(G_time,G_extend);

% simulation total time in counts
Simulation_Time = 15;
COUNT = Simulation_Time/sample_time;

% Step parameter
StepTime = 2;
StartTime = 0.4 * StepTime;
RestTime = 0.3 * StepTime;
DblTime = 0.3 * StepTime;
ZMPx_StepLength = 0.08;
ZMPy_StepLength = 0.056;

Ref_ZMPx = zeros(1,COUNT);
Ref_ZMPy = zeros(1,COUNT);
ZMPx = zeros(1,COUNT);
ZMPy = zeros(1,COUNT);
X = zeros(3,COUNT);
Y = zeros(3,COUNT);
Input_ux = zeros(1,COUNT);
Input_uy = zeros(1,COUNT);
Time = zeros(1,COUNT);

i=2;
for t=sample_time:sample_time:StepTime
    if(t<=StartTime)
        Ref_ZMPx(1,i) = 0;
        Ref_ZMPy(1,i) = 0;
    elseif(t<=StartTime + DblTime)
        a = smothstep(0,-ZMPy_StepLength/2,DblTime);
        tmp_time = t-StartTime;
        Ref_ZMPx(1,i) = 0;
        Ref_ZMPy(1,i) = a(1) + a(2)*tmp_time + a(3)*tmp_time^2 + a(4)*tmp_time^3 + a(5)*tmp_time^4;
    else
        Ref_ZMPx(1,i) = 0;
        Ref_ZMPy(1,i) = -ZMPy_StepLength/2;
    end
    i=i+1;
end
for t=sample_time:sample_time:StepTime
    if(t<=StartTime)
        Ref_ZMPx(1,i) = 0;
        Ref_ZMPy(1,i) = -ZMPy_StepLength/2;
    elseif(t<=StartTime + DblTime)
        ax = smothstep(0,ZMPx_StepLength,DblTime);
        ay = smothstep(-ZMPy_StepLength/2,ZMPy_StepLength/2,DblTime);
        tmp_time = t-StartTime;
        Ref_ZMPx(1,i) = ax(1) + ax(2)*tmp_time + ax(3)*tmp_time^2 + ax(4)*tmp_time^3 + ax(5)*tmp_time^4;
        Ref_ZMPy(1,i) = ay(1) + ay(2)*tmp_time + ay(3)*tmp_time^2 + ay(4)*tmp_time^3 + ay(5)*tmp_time^4;
    else
        Ref_ZMPx(1,i) = ZMPx_StepLength;
        Ref_ZMPy(1,i) = ZMPy_StepLength/2;
    end
    i=i+1;
end
for t=sample_time:sample_time:StepTime
    if(t<=StartTime)
        Ref_ZMPx(1,i) = ZMPx_StepLength;
        Ref_ZMPy(1,i) = ZMPy_StepLength/2;
    elseif(t<=StartTime + DblTime)
        a = smothstep(ZMPy_StepLength/2,0,DblTime);
        tmp_time = t-StartTime;
        Ref_ZMPx(1,i) = ZMPx_StepLength;
        Ref_ZMPy(1,i) = a(1) + a(2)*tmp_time + a(3)*tmp_time^2 + a(4)*tmp_time^3 + a(5)*tmp_time^4;
    else
        Ref_ZMPx(1,i) = ZMPx_StepLength;
        Ref_ZMPy(1,i) = 0;
    end
    i=i+1;
end

for j=i:COUNT
    Ref_ZMPx(1,j) = ZMPx_StepLength;
    Ref_ZMPy(1,j) = 0;
end

% for t =sample_time:sample_time:Simulation_Time
%     if(t<RestTime+Prv_Time-DblTime/2)
%         Ref_ZMPx(1,i) = 0;
%         Ref_ZMPy(1,i) = 0;
%     elseif(t<RestTime+Prv_Time+DblTime/2)
%         a = smothstep(0,-ZMPy_StepLength/2,DblTime);
%         tmp_time = t-(RestTime+Prv_Time-DblTime/2);
%         Ref_ZMPx(1,i) = 0;
%         Ref_ZMPy(1,i) = a(1) + a(2)*tmp_time + a(3)*tmp_time^2 + a(4)*tmp_time^3 + a(5)*tmp_time^4;
%     elseif(t<RestTime + StepTime + RestTime + Prv_Time - DblTime/2)
%         Ref_ZMPx(1,i) = 0;
%         Ref_ZMPy(1,i) = -ZMPy_StepLength/2;
%     elseif(t<RestTime + StepTime + RestTime + Prv_Time + DblTime/2)    
%         ax = smothstep(0,ZMPx_StepLength,DblTime);
%         ay = smothstep(-ZMPy_StepLength/2,ZMPy_StepLength/2,DblTime);
%         tmp_time = t-(RestTime + StepTime + RestTime + Prv_Time - DblTime/2);
%         Ref_ZMPx(1,i) = ax(1) + ax(2)*tmp_time + ax(3)*tmp_time^2 + ax(4)*tmp_time^3 + ax(5)*tmp_time^4;
%         Ref_ZMPy(1,i) = ay(1) + ay(2)*tmp_time + ay(3)*tmp_time^2 + ay(4)*tmp_time^3 + ay(5)*tmp_time^4;
%     elseif(t<RestTime + StepTime + RestTime + StepTime + RestTime + Prv_Time - DblTime/2)
%         Ref_ZMPx(1,i) = ZMPx_StepLength;
%         Ref_ZMPy(1,i) = ZMPy_StepLength/2;
%     elseif(t<RestTime + StepTime + RestTime + StepTime + RestTime + Prv_Time + DblTime/2)
%         ay = smothstep(ZMPy_StepLength/2,0,DblTime);
%         tmp_time = t-(RestTime + StepTime + RestTime + StepTime + RestTime + Prv_Time - DblTime/2);
%         Ref_ZMPx(1,i) = ZMPx_StepLength;
%         Ref_ZMPy(1,i) = ay(1) + ay(2)*tmp_time + ay(3)*tmp_time^2 + ay(4)*tmp_time^3 + ay(5)*tmp_time^4;        
%     elseif(t<RestTime + StepTime + RestTime + StepTime + RestTime + StepTime + RestTime + Prv_Time)
%         Ref_ZMPx(1,i) = ZMPx_StepLength;
%         Ref_ZMPy(1,i) = 0;
%     else
%         Ref_ZMPx(1,i) = ZMPx_StepLength;
%         Ref_ZMPy(1,i) = 0;
%     end
%     i=i+1;
% end


% initialize state

X(:,1) = zeros(3,1);
Y(:,1) = zeros(3,1);
ZMPx(1) = C*X(:,1);
ZMPy(1) = C*Y(:,1);
Inputx_tmp = 0;
Inputy_tmp = 0;

Ks = K_extend(1);
Kx = K_extend(2:4);

for i=1:COUNT-1
    Inputx_tmp = Inputx_tmp + (-Ks)*(ZMPx(i)-Ref_ZMPx(i));
    Inputy_tmp = Inputy_tmp + (-Ks)*(ZMPy(i)-Ref_ZMPy(i));
    Input_ux(i) = Inputx_tmp - Kx*X(:,i);
    Input_uy(i) = Inputy_tmp - Kx*Y(:,i);
    for j=1:N
        Input_ux(i) = Input_ux(i) - G_extend(j)*Ref_ZMPx(min(i+j,COUNT));
        Input_uy(i) = Input_uy(i) - G_extend(j)*Ref_ZMPy(min(i+j,COUNT));
    end
    X(:,i+1) = A*X(:,i) + B*Input_ux(i);
    Y(:,i+1) = A*Y(:,i) + B*Input_uy(i);
    ZMPx(i+1)= C*X(:,i+1);
    ZMPy(i+1)= C*Y(:,i+1);
    Time(i+1) = Time(i)+sample_time;
end

figure(2);
plot(Time,X(1,:));
hold on;
plot(Time,ZMPx(:));
hold on;
plot(Time,Ref_ZMPx(:));
grid on;

figure(3);
plot(Time,X(1,:));
hold on;
plot(Time,X(2,:));
hold on;
plot(Time,X(3,:));
grid on;

figure(4);
plot(Time,Y(1,:));
hold on;
plot(Time,ZMPy(:));
hold on;
plot(Time,Ref_ZMPy(:));
grid on;

figure(5);
plot(Time,Y(1,:));
hold on;
plot(Time,Y(2,:));
hold on;
plot(Time,Y(3,:));
grid on;

figure(6)
plot(X(1,:),Y(1,:));
grid on;

fid = fopen('preview_test1.txt','w');
fprintf(fid,"#pragma once \r\n");
fprintf(fid,"constexpr double TEST1_REFERENCE_ZMPX[] = ");
fprintf(fid,"{");
for i=1:COUNT
    fprintf(fid,"%.12f",Ref_ZMPx(1,i));
    if(i~=COUNT)
        fprintf(fid,",");
    end
end
fprintf(fid,"};");
fprintf(fid,"\r\n");

fprintf(fid,"constexpr double TEST1_REFERENCE_ZMPY[] = ");
fprintf(fid,"{");
for i=1:COUNT
    fprintf(fid,"%.12f",Ref_ZMPy(1,i));
    if(i~=COUNT)
        fprintf(fid,",");
    end
end
fprintf(fid,"};");
fprintf(fid,"\r\n");

fprintf(fid,"constexpr double TEST1_REFERENCE_INPUTX[] = ");
fprintf(fid,"{");
for i=1:COUNT
    fprintf(fid,"%.12f",Input_ux(i));
    if(i~=COUNT)
        fprintf(fid,",");
    end
end
fprintf(fid,"};");
fprintf(fid,"\r\n");

fprintf(fid,"constexpr double TEST1_REFERENCE_INPUTY[] = ");
fprintf(fid,"{");
for i=1:COUNT
    fprintf(fid,"%.12f",Input_uy(i));
    if(i~=COUNT)
        fprintf(fid,",");
    end
end
fprintf(fid,"};");
fprintf(fid,"\r\n");

fprintf(fid,"constexpr double TEST1_REFERENCE_STATEX1[] = ");
fprintf(fid,"{");
for i=1:COUNT
    fprintf(fid,"%.12f",X(1,i));
    if(i~=COUNT)
        fprintf(fid,",");
    end
end
fprintf(fid,"};");
fprintf(fid,"\r\n");

fprintf(fid,"constexpr double TEST1_REFERENCE_STATEX2[] = ");
fprintf(fid,"{");
for i=1:COUNT
    fprintf(fid,"%.12f",X(2,i));
    if(i~=COUNT)
        fprintf(fid,",");
    end
end
fprintf(fid,"};");
fprintf(fid,"\r\n");

fprintf(fid,"constexpr double TEST1_REFERENCE_STATEX3[] = ");
fprintf(fid,"{");
for i=1:COUNT
    fprintf(fid,"%.12f",X(3,i));
    if(i~=COUNT)
        fprintf(fid,",");
    end
end
fprintf(fid,"};");
fprintf(fid,"\r\n");

fprintf(fid,"constexpr double TEST1_REFERENCE_STATEY1[] = ");
fprintf(fid,"{");
for i=1:COUNT
    fprintf(fid,"%.12f",Y(1,i));
    if(i~=COUNT)
        fprintf(fid,",");
    end
end
fprintf(fid,"};");
fprintf(fid,"\r\n");

fprintf(fid,"constexpr double TEST1_REFERENCE_STATEY2[] = ");
fprintf(fid,"{");
for i=1:COUNT
    fprintf(fid,"%.12f",Y(2,i));
    if(i~=COUNT)
        fprintf(fid,",");
    end
end
fprintf(fid,"};");
fprintf(fid,"\r\n");

fprintf(fid,"constexpr double TEST1_REFERENCE_STATEY3[] = ");
fprintf(fid,"{");
for i=1:COUNT
    fprintf(fid,"%.12f",Y(3,i));
    if(i~=COUNT)
        fprintf(fid,",");
    end
end
fprintf(fid,"};");
fprintf(fid,"\r\n");

fprintf(fid,"constexpr double TEST1_REFERENCE_STATEZMPX[] = ");
fprintf(fid,"{");
for i=1:COUNT
    fprintf(fid,"%.12f",ZMPx(1,i));
    if(i~=COUNT)
        fprintf(fid,",");
    end
end
fprintf(fid,"};");
fprintf(fid,"\r\n");

fprintf(fid,"constexpr double TEST1_REFERENCE_STATEZMPY[] = ");
fprintf(fid,"{");
for i=1:COUNT
    fprintf(fid,"%.12f",ZMPy(1,i));
    if(i~=COUNT)
        fprintf(fid,",");
    end
end
fprintf(fid,"};");
fprintf(fid,"\r\n");

fclose(fid);