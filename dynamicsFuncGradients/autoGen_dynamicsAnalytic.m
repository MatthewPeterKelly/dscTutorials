function [F,Fz] = autoGen_dynamicsAnalytic(q1,q2,dq1,dq2,u0,u1,u2,empty)
%AUTOGEN_DYNAMICSANALYTIC
%    [F,FZ] = AUTOGEN_DYNAMICSANALYTIC(Q1,Q2,DQ1,DQ2,U0,U1,U2,EMPTY)

%    This function was generated by the Symbolic Math Toolbox version 6.2.
%    22-Sep-2015 19:46:00

t3 = q1-q2;
t2 = cos(t3);
t4 = t2.^2;
t5 = t4-2.0;
t6 = 1.0./t5;
t7 = dq1.^2;
t8 = sin(t3);
t9 = q1.*2.0;
t10 = dq2.^2;
t14 = q2.*2.0;
t11 = t9-t14;
t12 = sin(t11);
t13 = t2.*u2;
t15 = t8.*t10;
t16 = t7.*t12.*(1.0./2.0);
t17 = cos(q1);
t18 = t17.*u0.*(3.0./2.0);
t19 = q1-t14;
t20 = -q2+t9;
t21 = cos(t11);
t22 = 1.0./t5.^2;
t23 = u2.*2.0;
t24 = t7.*t8.*2.0;
t25 = cos(t20);
t26 = t25.*u0;
t27 = t10.*t12.*(1.0./2.0);
t28 = cos(q2);
t40 = t28.*u0;
t41 = t2.*u1;
t29 = t13+t23+t24+t26+t27-t40-t41;
F = [empty+t6.*(t13+t15+t16+t18-u1+u2-u0.*cos(q1-q2.*2.0).*(1.0./2.0));empty-t6.*t29];
if nargout > 1
    t30 = t2.*t10;
    t31 = t7.*t21;
    t32 = sin(t19);
    t33 = cos(t19);
    t34 = t13+t15+t16+t18-u1+u2-t33.*u0.*(1.0./2.0);
    t35 = t2.*t8.*t22.*t34.*2.0;
    t36 = t2.*t7.*2.0;
    t37 = sin(t20);
    t38 = t10.*t21;
    t39 = t8.*u1;
    Fz = [empty+t35+t6.*(t30+t31-t8.*u2+t32.*u0.*(1.0./2.0)-u0.*sin(q1).*(3.0./2.0));empty-t6.*(t36+t38+t39-t8.*u2-t37.*u0.*2.0)-t2.*t8.*t22.*t29.*2.0;empty-t35-t6.*(t30+t31-t8.*u2+t32.*u0);empty+t6.*(t36+t38+t39-t8.*u2-t37.*u0-u0.*sin(q2))+t2.*t8.*t22.*t29.*2.0;empty+dq1.*t6.*t12;empty-dq1.*t6.*t8.*4.0;empty+dq2.*t6.*t8.*2.0;empty-dq2.*t6.*t12;empty+t6.*(t17.*(3.0./2.0)-t33.*(1.0./2.0));empty-t6.*(t25-t28);empty-t6;empty+t2.*t6;empty+t6.*(t2+1.0);empty-t6.*(t2+2.0)];
end