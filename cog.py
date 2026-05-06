import numpy as np

def rot_x(a):
    return np.array([[1,0,0],[0,np.cos(a),-np.sin(a)],[0,np.sin(a),np.cos(a)]])

def rot_y(a):
    return np.array([[np.cos(a),0,np.sin(a)],[0,1,0],[-np.sin(a),0,np.cos(a)]])

def rot_z(a):
    return np.array([[np.cos(a),-np.sin(a),0],[np.sin(a),np.cos(a),0],[0,0,1]])

# World-frame CoG values currently in your URDF (these are WRONG frame)
cog_world = {
    'link_2': np.array([0.015751501,  -0.00022073221, 0.19913687]),
    'link_3': np.array([0.00030352597, 0.00004170388, 0.38074728]),
    'link_4': np.array([-0.0000021388, -0.0022290515,  0.51387207]),
    'link_5': np.array([-0.0000021268,  0.00018039922, 0.59028250]),
}

# Each joint: (translation in parent frame, rotation matrix)
# joint_1: at [0,0,0.126], axis Z  -> at q=0, R=I
# joint_2: at [0,0.069,0.033] in link_2 frame, axis Y -> at q=0, R=I
# joint_3: at [0.03,-0.0115,0.264] in link_3 frame, axis Y -> at q=0, R=I
# joint_4: at [0.195,-0.0575,0.03] in link_4 frame, axis X -> at q=0, R=I

# Build world transforms T = [R|t] for each link origin at q=0
# (all joint angles=0 so all R=I, transforms just accumulate translations)

R = np.eye(3)  # at q=0 all rotations are identity

# link_2 origin in world
t_link2 = np.array([0.0, 0.0, 0.126])
R_link2 = R.copy()

# link_3 origin in world = link_2_origin + R_link2 @ j2_offset
t_link3 = t_link2 + R_link2 @ np.array([0.0, 0.069, 0.033])
R_link3 = R_link2.copy()  # joint_2 axis=Y, q=0 so R=I

# link_4 origin in world
t_link4 = t_link3 + R_link3 @ np.array([0.03, -0.0115, 0.264])
R_link4 = R_link3.copy()

# link_5 origin in world
t_link5 = t_link4 + R_link4 @ np.array([0.195, -0.0575, 0.03])
R_link5 = R_link4.copy()

transforms = {
    'link_2': (t_link2, R_link2),
    'link_3': (t_link3, R_link3),
    'link_4': (t_link4, R_link4),
    'link_5': (t_link5, R_link5),
}

print("Corrected local frame CoG values for URDF:\n")
for link, cog_w in cog_world.items():
    t, Rot = transforms[link]
    # local = R^T @ (cog_world - link_origin)
    cog_local = Rot.T @ (cog_w - t)
    print(f'  {link}:')
    print(f'    <origin xyz="{cog_local[0]:.8f} {cog_local[1]:.8f} {cog_local[2]:.8f}" rpy="0 0 0"/>')
    print()