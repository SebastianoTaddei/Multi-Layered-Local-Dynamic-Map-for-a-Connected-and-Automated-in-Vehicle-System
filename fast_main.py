"""--------------------------------------------------------------------------
    This file is meant to handle the simulation of the scenario and time it
    for real-time validation.
   --------------------------------------------------------------------------"""

''' IMPORTS '''
# API files
import ldm.neo4j as neo4j
import ldm.utils as utils
import ldm.data.parse as parse
import draw_plots as draw_plots

# pandas to import data as a dataframe
import pandas as pd

# time to measure performance
import time as time

# tqdm to show a progress bar
from tqdm import tqdm, trange

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
sim_time = ego.index

# Initialise det_ID
utils.init_det_ID()
utils.init_paths()
utils.init_det_classes()
utils.init_conn_classes()
utils.init_lcpo()
utils.init_match_counters()

''' SIMULATION '''
# Connect to the database
driver = neo4j.connect_db()

# Perform multiple runs to measure performance
run_for = 1

# Initialise the time KPIs
tot_t   = 0
tot_os  = 0
tot_mos = 0

# Samples to process
start_sample = 0
end_sample   = 2000

# Print starting message
str_sim = ("Starting simulation: "
           "{run_for} run(s) with {samples} sample(s).").format

print(str_sim(run_for=run_for, samples=(end_sample - start_sample)))

# Simulate multiple times
for i in trange(run_for):
    # Empty database for consistent performance measurements
    neo4j.empty_db(driver)

    # Create constraints
    neo4j.create_uniqueness_constraints(driver)

    # Internal time containers
    int_os  = 0
    int_mos = 0

    # Loop over data
    for t in tqdm(sim_time[start_sample:end_sample]):
        # Start measuring time
        start_t = time.time()

        # Convert t to python datetime so that it's supported by neo4j
        py_t = t.to_pydatetime()

        # Parse the data
        with driver.session() as session:
            # Open the transaction
            tx = session.begin_transaction()

            # Parse ego data
            if t in ego.index:
                parse.ego_vehicle(tx, ego.loc[t], py_t)

            # Parse connection data
            if t in connections.index:
                parse.connection(tx, connections.loc[[t]], py_t)

            # Parse detection data
            if t in detections.index:
                (best_mp, mp_type,
                 coll_trajs,
                 likely_clothoids) = parse.detection(tx, detections.loc[[t]],
                                                     py_t)
            else:
                (best_mp, mp_type,
                 coll_trajs,
                 likely_clothoids) = parse.detection(tx,
                                                     pd.DataFrame({'A' : []}),
                                                     py_t)

            # Commit the transaction
            tx.commit()

            # Close the transaction
            tx.close()

        # Finish measuring time
        end_t = time.time()

        # Update the one step time
        int_os += end_t - start_t

        # Update the maximum one step time
        if end_t - start_t > int_mos:
            int_mos = end_t - start_t

    # Finish measuring
    end_t =  time.time()

    # Update the total times
    tot_t   += int_os
    tot_os  += int_os/len(sim_time[start_sample:end_sample])
    tot_mos += int_mos

# Close the connection to the database
neo4j.close_db(driver)

# Report performance
t_delta = sim_time[end_sample] - sim_time[start_sample]
sim_sec = t_delta.total_seconds()
avg_t   = tot_t/run_for
avg_os  = tot_os/run_for
avg_mos = tot_mos/run_for

# Average processing time full sim
str_t   = ("The average time needed to process "
           "{sim_sec:.3f} [s] was {avg_t:.3f} [s].").format
print(str_t(sim_sec=sim_sec, avg_t=avg_t))

# Average processing time one step
str_os   = ("The average time needed to process one step is "
           "{avg_os:.3f} [s].").format
print(str_os(avg_os=avg_os))

# Average maximum processing time one step
str_mos   = ("The average maximum time needed to process one step is "
           "{avg_mos:.3f} [s].").format
print(str_mos(avg_mos=avg_mos))

# Report matching performance, only use with one run!
end_str = ("Number of total matches: {0}.\n"
           "Number of correct matches: {1}.\n"
           "Number of maybe matches: {2}.\n"
           "Number of wrong matches: {3}.\n"
           "Number of unmatched matches: {4}.\n").format

print(end_str(utils.total_matches, utils.correct_matches, utils.maybe_matches,
              utils.wrong_matches, utils.unmatched_matches))