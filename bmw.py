import carla
import cv2
import numpy as np
import random
import time

# === Connect to CARLA ===
client = carla.Client('localhost', 3000)
client.set_timeout(10.0)
world = client.get_world()
blueprint_library = world.get_blueprint_library()


# === Spawn BMW vehicle ===
vehicle_bp = blueprint_library.find('vehicle.bmw.grandtourer')
vehicle_bp.set_attribute('color', '0,0,50')  # Dunkelblau in RGB
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
    ("Top-Down", carla.Transform(carla.Location(z=15), carla.Rotation(pitch=-90)))
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
            
        
            ### for the middle grid, center info image with position + time

            # Use top-down camera image for center tile
            top_down = images[8].copy()

            # Get info
            location = vehicle.get_location()
            timestamp = world.get_snapshot().timestamp.elapsed_seconds
            info_lines = [
                f"Position:",
                f"X: {location.x:.1f}  Y: {location.y:.1f}  Z: {location.z:.1f}",
                f"Time: {timestamp:.2f} s"
            ]

            # Overlay text
            font = cv2.FONT_HERSHEY_SIMPLEX
            scale = 0.6
            color = (255, 255, 255)
            line_height = 25
            for i, line in enumerate(info_lines):
                cv2.putText(top_down, line, (10, 30 + i * line_height), font, scale, color, 1, cv2.LINE_AA)

            black = top_down  # replace black tile with top-down + text
            # Get position and time
            location = vehicle.get_location()
            timestamp = world.get_snapshot().timestamp.elapsed_seconds

            # Format text
            pos_text = f"Position:\nX: {location.x:.1f}  Y: {location.y:.1f}  Z: {location.z:.1f}"
            time_text = f"Time: {timestamp:.2f} s"
            lines = pos_text.split("\n") + [time_text]

            # Render multiline text
            font = cv2.FONT_HERSHEY_SIMPLEX
            scale = 0.6
            color = (255, 255, 255)
            line_height = 25
            for i, line in enumerate(lines):
                cv2.putText(black, line, (10, 30 + i * line_height), font, scale, color, 1, cv2.LINE_AA)
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
