%% If testing this script directly, uncomment this section

% DH Parameters for UR5
% a = [0, -0.42500, -0.39225, 0, 0, 0];
% d = [0.089459, 0, 0, 0.10915, 0.09465, 0.0823];
% alpha = [pi/2, 0, 0, pi/2, -pi/2, 0];

% Set thetas for testing

% theta = [0, 0, 0, 0, 0, 0];
% theta = [3.8481, -0.8830, 1.2017, -0.3186, 2.2773, -1.5708];
% theta = [3.8481; -0.9367; 1.0309; -0.0942; 2.2773; -1.5708];
% theta = [3.8481, -0.8830, 1.2017, -0.3186, 2.2773, -1.5708];

%% 

function fk = forward_kinematics_ur5(theta, d, a, alpha)


    %This function computes the transformation matrix given theta, d, a, and alpha.
    T= @(theta, d, a, alpha)[...
            cos(theta), -cos(alpha)*sin(theta), sin(alpha)*sin(theta), a*cos(theta);...
            sin(theta), cos(alpha)*cos(theta), -sin(alpha)*cos(theta), a*sin(theta);...
            0, sin(alpha), cos(alpha), d;...
            0, 0, 0, 1 ...
        ];
    
    
    %Now we calculate transformation matrices
    
    T01 = T(theta(1), d(1), a(1), alpha(1));
    T12 = T(theta(2), d(2), a(2), alpha(2));
    T23 = T(theta(3), d(3), a(3), alpha(3));
    T34 = T(theta(4), d(4), a(4), alpha(4));
    T45 = T(theta(5), d(5), a(5), alpha(5));
    T56 = T(theta(6), d(6), a(6), alpha(6));
    
    %We also obtain the multiplied matrices
    T02 = T01 * T12;
    T03 = T02 * T23;
    T04 = T03 * T34;
    T05 = T04 * T45;
    T06 = T05 * T56;
    
    fk = T06;

end
