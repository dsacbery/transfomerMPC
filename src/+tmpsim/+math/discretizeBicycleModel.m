function [Ad, Bd, Ed, Cay, Day, day0] = discretizeBicycleModel(vehicle, cfg, vxUsed, TsMpc)
%DISCRETIZEBICYCLEMODEL Linear four-state bicycle model for lateral MPC.

validateattributes(vxUsed, {'double'}, {'real', 'finite', 'scalar', 'positive'});
validateattributes(TsMpc, {'double'}, {'real', 'finite', 'scalar', 'positive'});
if ~isstruct(cfg) || ~isfield(cfg, 'sample') || ~isfield(cfg.sample, 'Ts_mpc') ...
        || ~isFiniteScalar(cfg.sample.Ts_mpc) ...
        || abs(double(cfg.sample.Ts_mpc) - double(TsMpc)) > 1e-12
    error('tmpsim:InvalidBicycleModelConfig', ...
        'cfg.sample.Ts_mpc must match the supplied finite sample time.');
end
if ~isstruct(vehicle) || ~isfield(vehicle, 'mass') ...
        || ~isfield(vehicle, 'Iz') || ~isfield(vehicle, 'lf') ...
        || ~isfield(vehicle, 'lr') || ~isfield(vehicle, 'Cf') ...
        || ~isfield(vehicle, 'Cr')
    error('tmpsim:InvalidBicycleModelParameters', ...
        'vehicle must contain mass, Iz, lf, lr, Cf, and Cr.');
end
if any(~isfinite([vehicle.mass, vehicle.Iz, vehicle.lf, vehicle.lr, ...
        vehicle.Cf, vehicle.Cr])) || any([vehicle.mass, vehicle.Iz, ...
        vehicle.lf, vehicle.lr, vehicle.Cf, vehicle.Cr] <= 0)
    error('tmpsim:InvalidBicycleModelParameters', ...
        'vehicle parameters must be finite positive scalars.');
end

vx = max(double(vxUsed), 1.0);
m = double(vehicle.mass);
iz = double(vehicle.Iz);
lf = double(vehicle.lf);
lr = double(vehicle.lr);
cf = double(vehicle.Cf);
cr = double(vehicle.Cr);

% x = [e_y, e_psi, beta, r], input is front-wheel steering increment.
Ac = zeros(4, 4);
Bc = zeros(4, 1);
Ec = zeros(4, 1);
Ac(1, 2) = vx;
Ac(1, 3) = vx;
Ac(2, 4) = 1.0;
Ac(3, 3) = -(cf + cr) / (m * vx);
Ac(3, 4) = (cr * lr - cf * lf) / (m * vx^2) - 1.0;
Ac(4, 3) = (cr * lr - cf * lf) / (iz * vx);
Ac(4, 4) = -(cf * lf^2 + cr * lr^2) / (iz * vx);
Bc(3) = cf / (m * vx);
Bc(4) = cf * lf / (iz * vx);
Ec(2) = -vx;

augmented = expm([Ac, Bc, Ec; zeros(2, 6)] * double(TsMpc));
Ad = augmented(1:4, 1:4);
Bd = augmented(1:4, 5);
Ed = augmented(1:4, 6);

% a_y = vx * (beta_dot + r), evaluated from the continuous model.
Cay = [0.0, 0.0, vx * Ac(3, 3), vx * (Ac(3, 4) + 1.0)];
Day = vx * Bc(3);
day0 = 0.0;
end

function valid = isFiniteScalar(value)
valid = isnumeric(value) && isreal(value) && isscalar(value) ...
    && isfinite(value);
end
