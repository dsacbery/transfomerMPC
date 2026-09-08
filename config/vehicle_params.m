function vehicle = vehicle_params()
%VEHICLE_PARAMS Returns C-Class Hatchback nominal bicycle-model parameters.

vehicle.source.dataset = "C-Class, Hatchback";
vehicle.source.evidence_run = "day2_export_smoke";
vehicle.source.carsim_version = "2019.1";

vehicle.mass = 1501.0;
vehicle.Iz = 2192.089539;
vehicle.wheelbase = 2.910;
vehicle.lf = 1.049562958;
vehicle.lr = 1.860437042;

vehicle.tire.linearization = "small_slip_0p5_deg_at_static_load";
vehicle.tire.front_wheel_static_load = 4705.36546938348;
vehicle.tire.rear_wheel_static_load = 2654.52535561652;
vehicle.Cf = 159055.11037704;
vehicle.Cr = 92776.4754843415;
end
