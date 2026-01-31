now = self.get_clock().now()
dt = (now - self._prev_time).nanoseconds * 1e-9
dt = max(dt, 1e-4)

# pose error (world frame)
ex    = x_goal - x
ey    = y_goal - y
eyaw  = wrap_pi(yaw_goal - yaw)

err = np.array([ex, ey, eyaw])
derr = (err - self._prev_err) / dt

# PD (world frame)
vx_w = self.kp_xy  * ex   - self.kd_xy  * derr[0]
vy_w = self.kp_xy  * ey   - self.kd_xy  * derr[1]
wz   = self.kp_yaw * eyaw - self.kd_yaw * derr[2]

# world → base
c = math.cos(-yaw)
s = math.sin(-yaw)
vx_b = c * vx_w - s * vy_w
vy_b = s * vx_w + c * vy_w

# clamp
vx_b = clamp(vx_b, -self.max_vx, self.max_vx)
vy_b = clamp(vy_b, -self.max_vy, self.max_vy)
wz   = clamp(wz,   -self.max_wz, self.max_wz)

cmd.bvx = float(vx_b)
cmd.bvy = float(vy_b)
cmd.bwz = float(wz)

# update memory
self._prev_err = err
self._prev_time = now
