import math

def rasterize_sphere(R, cam_pos, cam_dir, cam_up, W, H, fov_y_deg):
    # Convert FOV to radians
    fov_y = math.radians(fov_y_deg)
    
    # Vector utilities
    def dot(a, b):
        return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]

    def cross(a, b):
        return (
            a[1]*b[2] - a[2]*b[1],
            a[2]*b[0] - a[0]*b[2],
            a[0]*b[1] - a[1]*b[0],
        )

    def normalize(v):
        mag = math.sqrt(dot(v, v))
        return (v[0]/mag, v[1]/mag, v[2]/mag)

    # Build camera basis (right-handed: x = right, y = up, z = back)
    forward = normalize(cam_dir)
    cam_z = (-forward[0], -forward[1], -forward[2])
    cam_x = normalize(cross(cam_up, cam_z))
    cam_y = cross(cam_z, cam_x)

    # Transform sphere center into camera space
    sphere_center = (0.0, 0.0, 0.0)
    rel = (
        sphere_center[0] - cam_pos[0],
        sphere_center[1] - cam_pos[1],
        sphere_center[2] - cam_pos[2],
    )
    C_x = dot(cam_x, rel)
    C_y = dot(cam_y, rel)
    C_z = dot(cam_z, rel)

    # If sphere is behind the camera, don't draw
    if C_z > 0:
        return ["_" * W for _ in range(H)]

    # Projection constants
    py = (H / 2) / math.tan(fov_y / 2)
    px = py * (W / H)
    cx, cy = W / 2, H / 2

    # Intersection constant
    C = C_x*C_x + C_y*C_y + C_z*C_z - R*R

    # Rasterize ASCII image
    image = []
    for j in range(H):
        v = (cy - j) / py
        a = C_x*C_x - C
        b = 2 * C_x * (v*C_y - C_z)
        c = (v*C_y - C_z)**2 - C*(v*v + 1)
        D = b*b - 4*a*c
        row = ['_'] * W
        if D >= 0 and a != 0:
            sqrtD = math.sqrt(D)
            u1 = (-b + sqrtD) / (2*a)
            u2 = (-b - sqrtD) / (2*a)
            x1 = cx + u1 * px
            x2 = cx + u2 * px
            xs = max(0, min(math.ceil(min(x1, x2)), W-1))
            xe = max(0, min(math.floor(max(x1, x2)), W-1))
            for x in range(xs, xe+1):
                row[x] = '#'
        image.append(''.join(row))
    return image

# Test cases
params = {
    'R': 2.0,
    'cam_dir': (0.0, 1.0, 0.0),
    'cam_up': (1.0, 0.0, 0.0),
    'W': 20,
    'H': 20,
    'fov_y_deg': 90.0
}

# Original camera (should see sphere)
cam1 = {'cam_pos': (-4.0, -10.0, 0.0)}
image1 = rasterize_sphere(**params, **cam1)

# Modified camera (sphere behind, no draw)
cam2 = {'cam_pos': (-4.0, 10.0, 0.0)}
image2 = rasterize_sphere(**params, **cam2)

# Display results
print("Camera at -4, -10, 0:")
for line in image1:
    print(line)
print("\nCamera at -4, 10, 0:")
for line in image2:
    print(line)