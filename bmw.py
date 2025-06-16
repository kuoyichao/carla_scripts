import carla
import cv2
import numpy as np
import random
import time

# === Connect to CARLA ===
client = carla.Client('localhost', 2000)
client.set_timeout(10.0)
world = client.get_world()
blueprint_library = world.get_blueprint_library()

# === Spawn BMW vehicle ===
vehicle_bp = blueprint_library.find('vehicle.bmw.grandtourer')
spawn_point = random.choice(world.get_map().get_spawn_points())
vehicle = world.spawn_actor(vehicle_bp, spawn_point)

# === Camera configurations ===
CAMERA_TRANSFORMS = [
    ("Front",       carla.Transform(carla.Location(z=2.5), carla.Rotation(yaw=0))),
    ("Rear",        carla.Transform(carla.Location(z=2.5), carla.Rotation(yaw=180))),
    ("Left",        carla.Transform(carla.Location(z=2.5), carla.Rotation(yaw=-90))),
    ("Right",       carla.Transform(carla.Location(z=2.5), carla.Rotation(yaw=90))),
    ("Front-Left",  carla.Transform(carla.Location(z=2.5), carla.Rotation(yaw=-45))),
    ("Front-Right", carla.Transform(carla.Location(z=2.5), carla.Rotation(yaw=45))),
    ("Rear-Left",   carla.Transform(carla.Location(z=2.5), carla.Rotation(yaw=-135))),
    ("Rear-Right",  carla.Transform(carla.Location(z=2.5), carla.Rotation(yaw=135))),
]

camera_bp = blueprint_library.find('sensor.camera.rgb')
camera_bp.set_attribute('image_size_x', '640')
camera_bp.set_attribute('image_size_y', '480')
camera_bp.set_attribute('fov', '90')

images = [None] * len(CAMERA_TRANSFORMS)
cameras = []

# === Camera callbacks ===
def make_callback(i):
    def callback(image):
        array = np.frombuffer(image.raw_data, dtype=np.uint8)
        array = array.reshape((image.height, image.width, 4))[:, :, :3]
        images[i] = array
    return callback

# === Spawn cameras ===
for i, (_, transform) in enumerate(CAMERA_TRANSFORMS):
    cam = world.spawn_actor(camera_bp, transform, attach_to=vehicle)
    cam.listen(make_callback(i))
    cameras.append(cam)

# === Control object ===
control = carla.VehicleControl()

# === Main loop ===
try:
    while True:
        if all(img is not None for img in images):
            h, w, _ = images[0].shape
            black = np.zeros((h, w, 3), dtype=np.uint8)
            grid = [
                images[4], images[0], images[5],  # Front-Left, Front, Front-Right
                images[2], black,      images[3],  # Left, Empty, Right
                images[6], images[1], images[7],  # Rear-Left, Rear, Rear-Right
            ]
            rows = [np.hstack(grid[i:i+3]) for i in range(0, 9, 3)]
            stacked = np.vstack(rows)
            cv2.imshow('BMW X5 Surround View (3x3)', stacked)

        # === Keyboard control ===
        key = cv2.waitKey(1)
        if key == ord('w'):
            control.throttle = min(control.throttle + 0.05, 1.0)
            control.brake = 0.0
        elif key == ord('s'):
            control.brake = min(control.brake + 0.05, 1.0)
            control.throttle = 0.0
        elif key == ord('a'):
            control.steer = max(control.steer - 0.05, -1.0)
        elif key == ord('d'):
            control.steer = min(control.steer + 0.05, 1.0)
        elif key == ord(' '):
            control = carla.VehicleControl()  # stop
        elif key == ord('q'):
            break
        else:
            # Slowly return steering to 0
            control.steer *= 0.9

        vehicle.apply_control(control)
        time.sleep(0.03)

finally:
    print("Cleaning up...")
    for cam in cameras:
        cam.stop()
        cam.destroy()
    vehicle.destroy()
    cv2.destroyAllWindows()
