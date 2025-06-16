import carla
import time

client = carla.Client("localhost", 2000)
client.set_timeout(10.0)

world = client.get_world()

# Set async mode (default anyway)
settings = world.get_settings()
if settings.synchronous_mode:
    settings.synchronous_mode = False
    world.apply_settings(settings)

blueprint_library = world.get_blueprint_library()

# Spawn a Tesla Model 3
vehicle_bp = blueprint_library.filter("model3")[0]
spawn_point = world.get_map().get_spawn_points()[0]
vehicle = world.try_spawn_actor(vehicle_bp, spawn_point)

# Attach RGB camera
camera_bp = blueprint_library.find("sensor.camera.rgb")
camera_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)

def process_image(image):
    image.save_to_disk("_out/%06d.png" % image.frame)

camera.listen(process_image)

print("Simulation running... saving camera images to ./_out/")
time.sleep(10)

# Cleanup
camera.stop()
camera.destroy()
vehicle.destroy()
print("Done.")
