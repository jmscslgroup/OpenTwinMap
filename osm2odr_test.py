import carla

osm_path = "SubsetSelection/osm_subset_corrected_lidar.osm"
odr_path = "SubsetSelection/osm_to_odr.xodr"

# Read the .osm data
f = open(osm_path, 'r')
osm_data = f.read()
f.close()

# Define the desired settings. In this case, default values.
settings = carla.Osm2OdrSettings()
# Set OSM road types to export to OpenDRIVE
settings.set_osm_way_types(["motorway", "motorway_link", "trunk", "trunk_link", "primary", "primary_link", "secondary", "secondary_link", "tertiary", "tertiary_link", "unclassified", "residential"])
settings.generate_traffic_lights = True
settings.all_junctions_with_traffic_lights = False
# Convert to .xodr
xodr_data = carla.Osm2Odr.convert(osm_data, settings)

# save opendrive file
f = open(odr_path, 'w')
f.write(xodr_data)
f.close()