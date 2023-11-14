"""--------------------------------------------------------------------------
    This file is meant to handle the simulation of the scenario and create a
    video of it.
   --------------------------------------------------------------------------"""

''' IMPORTS '''
# API files
import ldm.neo4j as neo4j
import ldm.utils as utils
import ldm.data.parse as parse
import draw_plots as draw_plots

# numpy to create empty arrays
import numpy as np

# pandas to import data as a dataframe
import pandas as pd

# matplotlib to plot data
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, FFMpegWriter

# tqdm to show a progress bar
from tqdm import tqdm

# os to get the current working directory
import os

''' DATA '''
# Get the current working directory
dirname  = os.path.dirname(__file__)

# Select the scenario
scenarios = ['urban', 'suburban', 'highway']
scenario  = scenarios[2]

# Import data from CSV files
ego         = pd.read_csv(os.path.join(dirname, 'ldm/data/csv/' + scenario + '/ego.csv'), index_col='time', parse_dates=True)
connections = pd.read_csv(os.path.join(dirname, 'ldm/data/csv/' + scenario + '/connections.csv'), index_col='time', parse_dates=True)
detections  = pd.read_csv(os.path.join(dirname, 'ldm/data/csv/' + scenario + '/detections.csv'), index_col='time', parse_dates=True)

# Create a time series to time the simulation
time = ego.index

# Initialise global variables
utils.init_det_ID()
utils.init_paths()
utils.init_det_classes()
utils.init_conn_classes()
utils.init_lcpo()
utils.init_match_counters()

''' SIMULATION '''
# Get the pixel measurement unit
px = 1/plt.rcParams['figure.dpi']

# Create the figure
fig, ax = plt.subplots(1, figsize=(1000*px, 1000*px))
fig.suptitle('LDM Simulation')

# Define an initialisation function for the animation
def initialise_animation():
    pass

# Connect to the database
driver = neo4j.connect_db()

# Empty database
neo4j.empty_db(driver)

# Create constraints
neo4j.create_uniqueness_constraints(driver)

# Samples to process
start_sample = 0
end_sample   = 2000 # len(time)

# Loop over data using the animation step
def animate(i):
    # Get current simulation time
    t = time[i + start_sample]

    # Convert t to python datetime so that it's supported by neo4j
    py_t = t.to_pydatetime()

    # Parse the data
    with driver.session() as session:
        # Open the transaction
        tx = session.begin_transaction()

        # Parse the ego data
        if t in ego.index:
            parse.ego_vehicle(tx, ego.loc[t], py_t)

        # Parse the connection data
        if t in connections.index:
            parse.connection(tx, connections.loc[[t]], py_t)

        # Parse the detection data
        if t in detections.index:
            (best_mp, mp_type,
             coll_trajs,
             likely_clothoids) = parse.detection(tx, detections.loc[[t]], py_t)
            ground_truth       = [detections.loc[[t]]['Lon'].to_list(),
                                  detections.loc[[t]]['Lat'].to_list()]
        else:
            (best_mp, mp_type,
             coll_trajs,
             likely_clothoids) = parse.detection(tx, pd.DataFrame({'A' : []}),
                                                 py_t)
            ground_truth       = [0, 0]

        # Commit the transaction
        tx.commit()

        # Close the transaction
        tx.close()

    # Retrieve the data
    with driver.session() as session:
        # Open the transaction
        tx = session.begin_transaction()

        # Plot the scene
        draw_plots.scene(tx, ax, mp_type, coll_trajs, ground_truth, likely_clothoids)

        # To plot additional figures increase the number of subplots
        # Uncomment to plot the database
        # draw_plots.database(tx, ax[0, 1], py_t)

        # Uncomment to plot the motion primitives
        # draw_plots.primitives(ax[1, 0], best_mp)

        # Uncomment to plot the ego data
        # draw_plots.ego_data(tx, ax[1, 1])

        # Commit the transaction
        tx.commit()

        # Close the transaction
        tx.close()

    # Uncomment to save a snapshot of the database
    # current_date = py_t.strftime('%Y-%m-%d_%H-%M-%S-%f')
    # neo4j.export_graphml(driver, current_date[:-3])

# Number of simulation steps = number of frames
tot_frames = end_sample - start_sample

# Print starting message
str_sim = ("Starting simulation: {tot_frames} frames.").format

print(str_sim(tot_frames=tot_frames))

# Run the animation
ani = FuncAnimation(fig, animate, init_func=initialise_animation,
                    frames=tqdm(range(tot_frames), initial=1), interval=1,
                    repeat=False)

# Save the animation as a .mp4 file
writermp4 = FFMpegWriter(fps=10)
ani.save(os.path.join(dirname, 'simulations.nosync/' + scenario + '.mp4'), writer=writermp4)
plt.close()

# Close the connection to the database
neo4j.close_db(driver)

# Print ending message
end_str = ("Simulation ended.\n"
           "Number of total matches: {0}.\n"
           "Number of correct matches: {1}.\n"
           "Number of maybe matches: {2}.\n"
           "Number of wrong matches: {3}.\n"
           "Number of unmatched matches: {4}.\n").format

print(end_str(utils.total_matches, utils.correct_matches, utils.maybe_matches,
              utils.wrong_matches, utils.unmatched_matches))
