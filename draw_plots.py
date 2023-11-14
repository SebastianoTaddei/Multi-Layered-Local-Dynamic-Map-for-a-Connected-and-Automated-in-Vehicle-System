"""--------------------------------------------------------------------------
    This file is meant to handle the creation of the plots for the GUI.
   --------------------------------------------------------------------------"""

''' IMPORTS '''
# utils to have access to global variables
import ldm.utils as utils

# networkx to plot the database
import networkx as nx

# matplotlib to plot data
import matplotlib.pyplot as plt

# imports required to download OSM map tiles
import math
from io import BytesIO
from PIL import Image
import requests
from itertools import product
import os

# pymap3d to convert from ENU to Geodetic
import pymap3d as pm

# numpy to handle arrays
import numpy as np

''' FUNCTIONS '''
# Scene plot
def scene(tx, ax, mp_type, coll_trajs, ground_truth, likely_clothoids):
    # Get the ego, the detections, and the osm nodes
    result = tx.run('''
                    OPTIONAL MATCH (a:AutonomousVehicle:L4 {ID:11112000})
                    OPTIONAL MATCH (n1_a:OSMNode)-[:WAS_ON]-(a)-[:IS_ON]-(n2_a:OSMNode)

                    OPTIONAL MATCH (d:Detection)
                    OPTIONAL MATCH (n1_d:OSMNode)-[:WAS_ON]-(d)-[:IS_ON]-(n2_d:OSMNode)

                    OPTIONAL MATCH (n:OSMNode)

                    RETURN a.Lon, a.Lat,
                           [n1_a.Lon, n1_a.Lat],
                           [n2_a.Lon, n2_a.Lat],
                           collect(distinct [d.Lon, d.Lat, d.type,
                                             [n1_d.Lon, n1_d.Lat],
                                             [n2_d.Lon, n2_d.Lat]]),
                           collect(distinct [n.Lon, n.Lat])
                    LIMIT 1
                    ''')
    
    # Clear the figure
    ax.clear()

    # Get the data
    data = result.single().values()

    # Plot the OSM data
    osm_data = np.asarray(data[-1])
    if osm_data.all(axis=(0, 1)):
        # Plot the data
        ax.scatter(osm_data[:, 0], osm_data[:, 1], c='green')

    # Parse the ego data
    ego_node = data[0:4]

    if ego_node[0] and ego_node[1]:
        # Plot the ego data
        ax.scatter(ego_node[0], ego_node[1], c='orange')
        ax.text(ego_node[0], ego_node[1], 'EgoVehicle')

        if ego_node[2] and ego_node[3]:
            # Plot the WAS_ON and IS_ON relationships
            ax.plot([ego_node[0], ego_node[2][0]], [ego_node[1], ego_node[2][1]], c='blue', linestyle='dashed')
            ax.plot([ego_node[0], ego_node[3][0]], [ego_node[1], ego_node[3][1]], c='purple', linestyle='dashed')

        ''' OSM BASEMAP '''
        # Adapted from https://bryanbrattlof.com/adding-openstreetmaps-to-matplotlib/

        # Basemap extent
        box_offset = 1e-3
        top, bot = ego_node[1] + box_offset, ego_node[1] - box_offset
        lef, rgt = ego_node[0] - box_offset, ego_node[0] + box_offset

        # Check if basemap already exists, download it otherwise
        dirname  = os.path.dirname(__file__)
        filename = os.path.join(dirname,
                                'ldm/data/osm.nosync/basemap/basemap_'
                                + str(ego_node[1]) + '-'
                                + str(ego_node[0])
                                + '.png')

        if not os.path.exists(filename):
            # URL string framework
            URL = "https://tile.openstreetmap.org/{z}/{x}/{y}.png".format

            # Standard OSM tile size
            TILE_SIZE = 256

            # Mercator projection
            def point_to_pixels(lon, lat, zoom):
                """convert gps coordinates to web mercator"""
                r   = math.pow(2, zoom) * TILE_SIZE
                lat = math.radians(lat)

                x = int((lon + 180.0) / 360.0 * r)
                y = int((1.0 - math.log(math.tan(lat) + (1.0 / math.cos(lat))) / math.pi) / 2.0 * r)

                return x, y

            # Details of the map
            zoom = 18

            # Bounding box
            x0, y0 = point_to_pixels(lef, top, zoom)
            x1, y1 = point_to_pixels(rgt, bot, zoom)

            # Bounding box rounded to a multiple of the title size
            x0_tile, y0_tile = int(x0 / TILE_SIZE), int(y0 / TILE_SIZE)
            x1_tile, y1_tile = math.ceil(x1 / TILE_SIZE), math.ceil(y1 / TILE_SIZE)

            # Avoid downloading too many tiles
            assert (x1_tile - x0_tile) * (y1_tile - y0_tile) < 50, "That's too many tiles!"

            # Full size image we'll add tiles to
            basemap = Image.new('RGB', (
                (x1_tile - x0_tile) * TILE_SIZE,
                (y1_tile - y0_tile) * TILE_SIZE))

            # Loop through every tile inside our bounded box
            for x_tile, y_tile in product(range(x0_tile, x1_tile), range(y0_tile, y1_tile)):
                with requests.get(URL(x=x_tile, y=y_tile, z=zoom)) as resp:
                    tile_img = Image.open(BytesIO(resp.content))

                # Add each tile to the full size image
                basemap.paste(
                    im=tile_img,
                    box=((x_tile - x0_tile) * TILE_SIZE, (y_tile - y0_tile) * TILE_SIZE))

            # Top left corner of the image
            x, y = x0_tile * TILE_SIZE, y0_tile * TILE_SIZE

            # Crop the image to the initial wanted size
            basemap = basemap.crop((
                int(x0 - x),  # left
                int(y0 - y),  # top
                int(x1 - x),  # right
                int(y1 - y))) # bottom

            # Save image
            basemap.save(filename)
        else:
            # Open already saved image
            basemap = Image.open(filename)

    # Plot the detection data
    for lon, lat, type, n1, n2 in data[4]:
        if lon and lat:
            # Plot the data
            ax.scatter(lon, lat, c='black')
            ax.text(lon, lat, type)

            if n1 and n2:
                # Plot the WAS_ON and IS_ON relationships
                ax.plot([lon, n1[0]], [lat, n1[1]], c='blue', linestyle='dashed')
                ax.plot([lon, n2[0]], [lat, n2[1]], c='purple', linestyle='dashed')

    # Plot the likely clothoids and the motion primitive type
    for likely_clothoid in likely_clothoids:
        # Sample points
        xs, ys = likely_clothoid[0].SampleXY(100)

        # Containers
        lats, lons = [], []

        # For each point
        for x, y in zip(xs, ys):
            # Convert from ENU to Geodetic
            # Assume zero height because of the tangential plane approximation
            lat, lon, _ = pm.enu2geodetic(x, y, 0,
                                          *utils.lcpo[0:3])

            # Fill the containers
            lats.append(lat)
            lons.append(lon)

        # Plot the clothoid
        ax.plot(lons, lats, c='grey', linestyle='dashdot')
        ax.text(lons[0], lats[1] - 4e-5, likely_clothoid[1], c='magenta')

    # Plot the possible collision trajectories
    for ego_traj, det_traj in coll_trajs:
        # Containers
        ego_lats, ego_lons = [], []
        det_lats, det_lons = [], []

        # For each point in the ego trajectory
        for ego_x, ego_y in zip(ego_traj[:, 0], ego_traj[:, 1]):
            # Convert from ENU to Geodetic
            # Assume zero height because of the tangential plane approximation
            ego_lat, ego_lon, _ = pm.enu2geodetic(ego_x, ego_y, 0,
                                                  *utils.lcpo[0:3])

            # Fill the containers
            ego_lats.append(ego_lat)
            ego_lons.append(ego_lon)

        # For each point in the detection trajectory
        for det_x, det_y in zip(det_traj[:, 0], det_traj[:, 1]):
            # Convert from ENU to Geodetic
            # Assume zero height because of the tangential plane approximation
            det_lat, det_lon, _ = pm.enu2geodetic(det_x, det_y, 0,
                                                  *utils.lcpo[0:3])

            # Fill the containers
            det_lats.append(det_lat)
            det_lons.append(det_lon)

        # Plot the trajectories
        ax.plot(ego_lons, ego_lats, c='red', linestyle='dotted')
        ax.plot(det_lons, det_lats, c='brown', linestyle='dotted')

    # Plot the type of motion the ego should do
    if mp_type and ego_node[0] and ego_node[1]:
        ax.text(ego_node[0], ego_node[1] - 4e-5, mp_type, c='red')

    # Plot the detections ground truth
    ax.scatter(ground_truth[0], ground_truth[1], c='blue')

    # General figure parameters
    ax.set_title('Scene')
    ax.axis('off')

    # Plot the basemap
    ax.imshow(basemap, extent=(lef, rgt, bot, top))

# Database plot
def database(tx, ax, timestamp):
    # Get all nodes directly connected to the ego and their relationships
    result = tx.run('''
                    OPTIONAL MATCH (a:AutonomousVehicle:L4 {ID:11112000})
                    OPTIONAL MATCH (a)-[r]-(n)
                    WHERE a.timestamp = n.timestamp

                    RETURN id(a), labels(a), id(n), labels(n), id(r), type(r)
                    ''')

    # Create graph
    G = nx.MultiDiGraph()

    # Create labels containers
    node_labels = dict()
    edge_labels = dict()

    for record in result:
        node = record.values()

        # Add nodes and relationships
        both_added = True

        if node[0] and node[1]:
            G.add_node(node[0], labels=node[1])
            node_labels[node[0]] = node[1]
        else:
            both_added = False

        if node[2] and node[3]:
            G.add_node(node[2], labels=node[3])
            node_labels[node[2]] = node[3]
        else:
            both_added = False

        if both_added and node[4] and node[5]:
            G.add_edge(node[0], node[2], key=node[4], type=node[5])
            edge_labels[(node[0], node[2])] = node[5]


    # Create position layout
    pos = nx.spring_layout(G)

    # Clear the figure
    ax.clear()

    # Plot the graph
    nx.draw(G, pos=pos, ax=ax)
    nx.draw_networkx_labels(G, pos=pos, ax=ax, labels=node_labels)
    nx.draw_networkx_edge_labels(G, pos=pos, ax=ax, edge_labels=edge_labels)

    # General figure parameters
    ax.set_title('Database')
    ax.axis('off')

    # Add left and right margins to avoid cutting of labels
    x_values, y_values = zip(*pos.values())
    x_max              = max(x_values)
    x_min              = min(x_values)
    x_margin           = (x_max - x_min) * 0.30

    if not x_min - x_margin == x_max + x_margin == 0:
        ax.set_xlim(x_min - x_margin, x_max + x_margin)

# Ego data
def ego_data(tx, ax):
    # Get all nodes directly connected to the ego and their relationships
    result = tx.run('''
                    OPTIONAL MATCH (n:AutonomousVehicle:L4 {ID:11112000})

                    OPTIONAL MATCH (n)-[n_r1:IS_DETECTED]-(d:Detection)-[d_r1:WILL_BE]-(p:Prediction)

                    RETURN properties(n) as ego_vehicle,
                           collect(properties(d)) as detections,
                           collect(type(n_r1)) as d_rel_types,
                           collect(properties(n_r1)) as d_rel_props,
                           collect(properties(p)) as predictions,
                           collect(type(d_r1)) as p_rel_types
                    ''')

    # Clear the figure
    ax.clear()

    # Parse the ego data only once
    data = result.single().values()

    # Initialise containers with the ego data
    if data[0]:
        cellText  = [[str(data[0]['Lat']), str(data[0]['Lon']), '/']]
        rowLabels = ['Ego vehicle']
    else:
        cellText  = []
        rowLabels = []

    # Parse the rest of the data
    for (detection, d_rel_type, d_rel_prop,
         prediction, p_rel_type) in zip(data[1], data[2], data[3], data[4],
                                        data[5]):
        if detection:
            cellText.append([str(detection['Lat']),
                             str(detection['Lon']),
                             d_rel_type])
            rowLabels.append(d_rel_prop['type'])

        if prediction:
            cellText.append([str(prediction['Lat']),
                             str(prediction['Lon']),
                             p_rel_type])
            rowLabels.append('-- Prediction')


    # Create the table
    data_table = ax.table(cellText=cellText,
                          cellLoc='center',
                          rowLabels=rowLabels,
                          colLabels=['Latitude', 'Longitude', 'Relationship'],
                          loc='center')

    # Scale the table to better fit the data
    data_table.scale(1, 1.5)

    # General figure parameters
    ax.set_title('Data')
    ax.axis('off')

# Motion primitives
def primitives(ax, mp):
    # Clear the figure
    ax.clear()

    if mp.size != 0:
        # Plot the best primitives
        ax.plot(mp[:, 0], mp[:, 1],
                mp[:, 0], mp[:, 2],
                mp[:, 0], mp[:, 3])
    else:
        # Plot nothing
        ax.plot(0, 0, 0, 0, 0, 0)

    # General figure parameters
    ax.set_title('Motion primitives')
    ax.set_xlabel('Distance [m]')
    ax.legend(['Velocity', 'Acceleration', 'Jerk'])
