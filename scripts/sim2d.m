g = 9.80665;
% 1.2 kg/m^3 atmospheric density
m = 0.61; % kilograms
I = 0.01; % inertia; 2D sim - disallow yaw rotation

% initial conditions
x0 = [0.0; 1.0];
xdot0 = [0.0; 0.0];
theta0 = -1.0;
thetadot0 = 0.0;
X0 = [x0; xdot0; theta0; thetadot0];

% final conditions
xdes = [1.0; 1.0];
xdotdes = [0.0; 0.0];

% attitude control only
attOnly = 0;
force_att = m * g;
theta_att = 0.0;
thetadot_att = 0.0;

% gains kx = diag([ 20.0, 20.0 ], 0);
kxdot = diag([ 12.0, 12.0 ], 0);
kR = 40.0;
komega = 20.0;

tspan = 0:0.01:10;
[t,y] = ode45(@(t, X) sim2dfunc(t, X, ...
                                g, m, I, ...
                                xdes, xdotdes, ...
                                attOnly, force_att, theta_att, thetadot_att, ...
                                kx, kxdot, kR, komega), tspan, X0);

% figure;
% plot(t, y(:, 1), t, y(:, 2), t, y(:, 5));
% 
% figure;
% plot(t, y(:, 3), t, y(:, 4), t, y(:, 6));

thetades = zeros(size(t));
omegades = zeros(size(t));
for i = 1:length(t)
    [~, thetades(i), omegades(i)] = sim2dfunc(t, y(i, :)', ...
                                              g, m, I, ...
                                              xdes, xdotdes, ...
                                              attOnly, force_att, theta_att, thetadot_att, ...
                                              kx, kxdot, kR, komega);
end
                                        
figure;
plot(t, thetades, ...
     t, y(:, 5), ...
     t(1:end-1), diff(thetades)./diff(t), ...
     t, omegades, ...
     t, y(:, 6));
legend('thetades', 'theta', 'diff thetades', 'omegades', 'omega');
ylabel({'thrust vector angle (rad) / angular velocity (rad/s)'});
xlabel('time (s)');
xlim([0 5]);
ylim([-1.5 5]);

figure;
plot(t, y(:, 1), ...
     t, y(:, 2));
legend('x', 'z');