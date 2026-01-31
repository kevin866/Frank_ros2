from geometry_msgs.msg import TwistStamped

msg = TwistStamped()
msg.header.frame_id = "world"   # or base_link (VERY IMPORTANT)
msg.twist.linear.x = 0.0
msg.twist.linear.y = 0.0
msg.twist.linear.z = 0.05   # 5 cm/s upward
msg.twist.angular.x = 0.0
msg.twist.angular.y = 0.0
msg.twist.angular.z = 0.0
