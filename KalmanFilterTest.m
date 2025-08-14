% Kalman filter
%clear all
clc

% step 10 steps per second avg total 100 sec
h = 0.1;
% ident matrix
I = [ 1 0 0 0
      0 1 0 0
      0 0 1 0
      0 0 0 1 ];
% cov
Q = 10^-2 * I;
R = 10^-5 * I;

% initial con
q0 = accl2Quat(0,0,1);

% plotting
figure;

tiledlayout(3,1);
nexttile;

phi_line_measure = animatedline;
phi_line_measure.Color = '[0.01 0.7 0.99]';

phi_line_predict = animatedline;
phi_line_predict.Color = '[0.01 0.99 0.35]';
hold on

nexttile;

theta_line_measure = animatedline;
theta_line_measure.Color = '[0.01 0.7 0.99]';

theta_line_predict = animatedline;
theta_line_predict.Color = '[0.01 0.99 0.35]';
hold on

nexttile;

psi_line_measure = animatedline;
psi_line_measure.Color = '[0.01 0.7 0.99]';

psi_line_predict = animatedline;
psi_line_predict.Color = '[0.01 0.99 0.35]';
hold on

% loop through all data point steps in workspace
for k = 1:100

    % predictor inputs
    w1 = imu_data_gyro(k,1); %p rad/s
    w2 = imu_data_gyro(k,2); %q
    w3 = imu_data_gyro(k,3); %r

    % state space
    A = h/2 * [ 2/h    w3  -w2  w1
                -w3   2/h   w1  w2
                 w2   -w1  2/h  w3
                -w1   -w2  -w3  2/h];
    
    % initial condition check
    if k == 1
       q = q0;
       P = 0;
    end
        
    % estimate
    qp = A * q;
    % error cov
    B = getB(w1, w2, w3);
    Pp = B * P * B' + Q;

    % kalman gain
    K = Pp * I' * inv(I * Pp * I' + R);
    % corrector
    measureQuat = accl2Quat(imu_data_acc(k,1), imu_data_acc(k,2), imu_data_acc(k,3));
    % estimate
    q = qp + K * (measureQuat - I * qp);
    P = Pp - K * I * Pp;
    
    eul = quat2Euler(q');
    
    % plotting points on each graph
    addpoints(phi_line_measure, k, imu_data_rad(k,11));
    addpoints(phi_line_predict, k, -eul(:,2));
    
    addpoints(theta_line_measure, k, imu_data_rad(k,12));
    addpoints(theta_line_predict, k, eul(:,1));

    addpoints(psi_line_measure, k, imu_data_rad(k,13));
    addpoints(psi_line_predict, k, eul(:,3));

    drawnow;

end

hold off;



% functions
function [quat] = accl2Quat(accl_x, accl_y, accl_z)

    theta = asin(accl_x / 9.8);
    phi = atan(accl_y / accl_z);
    psi = 0;
    
    quat = [  sin(phi/2)*cos(theta/2)*cos(psi/2) - cos(phi/2)*sin(theta/2)*sin(psi/2)
              sin(phi/2)*cos(theta/2)*sin(psi/2) + cos(phi/2)*sin(theta/2)*cos(psi/2)
             -sin(phi/2)*sin(theta/2)*cos(psi/2) + cos(phi/2)*cos(theta/2)*sin(psi/2)
              sin(phi/2)*sin(theta/2)*sin(psi/2) + cos(phi/2)*cos(theta/2)*cos(psi/2) ];

end

function [Eul] = quat2Euler(Qt)

    q1 = Qt(1);
	q2 = Qt(2);
	q3 = Qt(3);
	q4 = Qt(4); 
    
    a11 = q1^2 - q2^2 - q3^2 + q4^2;
	a12 = 2 * ( q1 * q2 + q3 * q4 );
	a13 = 2 * ( q1 * q3 - q2 * q4 );
	a21 = 2 * ( q1 * q2 - q3 * q4 );
	a22 = q2^2 - q1^2 - q3^2 + q4^2;
	a23 = 2 * ( q2 * q3 + q1 * q4 );
	a31 = 2 * ( q1 * q3 + q2 * q4 );
	a32 = 2 * ( q2 * q3 - q1 * q4 );
	a33 = q3^2 - q1^2 - q2^2 + q4^2;

	DCM = [	a11 a12 a13;
	 		a21 a22 a23;
	 		a31 a32 a33];

    psi 	= atan2( a12, a11 );
	theta 	= atan2( -a13, sqrt( a11 * a11 + a12 * a12 ) );
	phi 	= atan2( a23, a33 );

    Eul = [phi, theta, psi];

end

function [B]= getB(w1,w2,w3)

    B =[ 0        0.5*w3  -0.5*w2     0.5*w1 
        -0.5*w3   0        0.5*w1     0.5*w2 
         0.5*w2  -0.5*w1    0         0.5*w3
        -0.5*w1  -0.5*w2  -0.5*w3     0      ];
end