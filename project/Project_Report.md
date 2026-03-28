



pid_steer.Init(0.30, 0.001, 4.0, 1.0, -1.0);   // Steer: Kp, Ki, Kd
pid_throttle.Init(0.60, 0.0, 0.05, 1.0, -1.0); // Throttle

Start with reasonable gains, for example:
Kp = 0.25 ~ 0.35
Kd = 3.0 ~ 8.0
Ki = 0.0005 ~ 0.005 (small — integral can cause oscillation)