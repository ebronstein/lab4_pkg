function dynamics = dynamics_gen(q,dq,K,D,Tau)
%DYNAMICS_GEN
%    DYNAMICS = DYNAMICS_GEN(Q,DQ,K,D,TAU)

%    This function was generated by the Symbolic Math Toolbox version 8.2.
%    06-May-2019 17:35:03

t6 = q.*1.0e4;
t2 = t6+1.0;
t3 = t2.^2;
t4 = q+1.0e-4;
t5 = sin(t4);
t7 = t3.^2;
t8 = cos(t4);
t9 = q.*1.2348e8;
t10 = t8.*1.2348e12;
t11 = t5.*1.2348e8;
t12 = q.^2;
t13 = t12.*6.174e11;
t16 = q.*t5.*1.2348e12;
t14 = t9-t10-t11+t13-t16+1.234800006174e12;
t15 = 1.0./t14;
dynamics = [dq;Tau.*t7.*t15-t7.*t15.*(K.*q-(1.0./t2.^2.*(t5.*-1.44207e9+t8.*1.44207e5+q.*t8.*1.44207e9))./1.0e3)-dq.*t7.*t15.*(D-dq.*1.0./t2.^5.*(q.*2.0e4-t5.*4.0e4-t8.*3.99999999e8+t12.*1.0e8-q.*t5.*4.0e8+q.*t8.*2.0e4+t8.*t12.*1.0e8+4.00000001e8).*6.174e7)];
