function gradu = singletrack_gradu_f_3o_nodert_noa_cal_f(in1,in2,in3)
%SINGLETRACK_GRADU_F_3O_NODERT_NOA_CAL_F
%    GRADU = SINGLETRACK_GRADU_F_3O_NODERT_NOA_CAL_F(IN1,IN2,IN3)

%    This function was generated by the Symbolic Math Toolbox version 9.0.
%    05-Dec-2023 19:07:13

B_f = in3(10,:);
C_f = in3(11,:);
E_f = in3(12,:);
Fzf = in3(18,:);
I_z = in3(4,:);
L_f = in3(2,:);
R_wheel = in3(1,:);
Tf = in2(:,1);
beta = in1(6,:);
delta = in2(:,3);
m = in3(5,:);
miu = in3(15,:);
r = in1(5,:);
v = in1(4,:);
t2 = cos(beta);
t3 = cos(delta);
t4 = sin(beta);
t5 = sin(delta);
t6 = L_f.*r;
t7 = B_f.^2;
t8 = 1.0./I_z;
t10 = 1.0./R_wheel;
t11 = -delta;
t12 = 1.0./m;
t13 = 1.0./v;
t9 = t4.*v;
t14 = 1.0./t2;
t15 = beta+t11;
t16 = cos(t15);
t17 = sin(t15);
t18 = t6+t9;
t19 = t13.*t14.*t18;
t20 = atan(t19);
t21 = -t20;
t22 = delta+t21;
t23 = t22.^2;
t24 = B_f.*t22;
t25 = atan(t24);
t26 = -t24;
t27 = t7.*t23;
t28 = t27+1.0;
t36 = t25+t26;
t37 = -E_f.*(t24-t25);
t29 = 1.0./t28;
t38 = t24+t37;
t30 = B_f.*t29;
t39 = atan(t38);
t40 = t38.^2;
t31 = -t30;
t41 = C_f.*t39;
t44 = t40+1.0;
t32 = B_f+t31;
t42 = cos(t41);
t43 = sin(t41);
t45 = 1.0./t44;
t33 = E_f.*t32;
t34 = -t33;
t35 = B_f+t34;
gradu = reshape([0.0,0.0,0.0,    0.0,0.0,0.0,      0.0,0.0,0.0            ,t10.*t12.*t16,t2.*t10.*t12,t12.*(Tf.*t10.*t17-Fzf.*miu.*t16.*t43+C_f.*Fzf.*miu.*t17.*t35.*t42.*t45),                L_f.*t5.*t8.*t10,0.0,L_f.*t8.*(Tf.*t3.*t10-Fzf.*miu.*t5.*t43+C_f.*Fzf.*miu.*t3.*t35.*t42.*t45),            -t10.*t12.*t13.*t17,-t4.*t10.*t12.*t13,t12.*t13.*(Tf.*t10.*t16+Fzf.*miu.*t17.*t43+C_f.*Fzf.*miu.*t16.*t35.*t42.*t45)],[3,6]);
