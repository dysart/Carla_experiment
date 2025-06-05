import carla

# Connect to the CARLA server
client = carla.Client('localhost', 2000)
client.set_timeout(30.0)

# Load the desired map by name (e.g., Town12)
world = client.load_world('Town12')

# Turn off the building layer
world.unload_map_layer(carla.MapLayer.Buildings)
# To turn buildings back on, use:
# world.load_map_layer(carla.MapLayer.Buildings)

# Unload as many non-essential map layers as possible to reduce system load

world.unload_map_layer(carla.MapLayer.ParkedVehicles)
world.unload_map_layer(carla.MapLayer.Particles)
world.unload_map_layer(carla.MapLayer.Foliage)

world.unload_map_layer(carla.MapLayer.Decals)
world.unload_map_layer(carla.MapLayer.Ground)
world.unload_map_layer(carla.MapLayer.Walls)
world.unload_map_layer(carla.MapLayer.Props)
world.unload_map_layer(carla.MapLayer.All)

# Print the name of the currently loaded map for confirmation
print('Current map:', world.get_map().name)