#####################################
# imports
# PREQUISITES: install plot.ly
#####################################
import plotly.graph_objs as go
import plotly.offline as offline
from plotly import tools
import numpy as np

#####################################
# config
#####################################

#define the percentile to show as result subset, 1-100%
perc_p = 100
# -1 combined normalized error
# -2 avg orientation error
# -3 avg location error
perc_data_ind = -1 
#show results with / without SEM
sem = 1

#advanced config: colum <-> axis mapping
x_ind = 2
y_ind = 4
z_ind = 1

#####################################
# data preparation
#####################################

overview = np.genfromtxt('sim_overview.csv', delimiter=',')

with open('sim_overview_h.txt') as header:
    text = header.readline()
    headers = text.split(',')

    
results = np.ndarray((np.size(overview,0),np.size(overview,1) + 2), dtype=float)    

for i in range(len(overview)):
    task_i = int(i)+1
    if(int(overview[i,-1]) == 1 ):
        errors = np.genfromtxt('task{}_errors.csv'.format(task_i), delimiter=',')
        avg_loc_error = errors[-1, 2]
        avg_ori_error = errors[-1, 3]
        
        results[i] = np.concatenate((overview[i], [avg_loc_error, avg_ori_error]), 0)
        
    elif (int(overview[i,-1]) == 2 ):
        results[i] = np.concatenate((overview[i], [0, 0]), 0)
        
nc_results = results [results[:,-1] == 0]
results = results [results[:,-1] > 0]
results = results[results[:,3] == sem] #filter sem results
 
#error normalization   
norm_avg_loc_err = (results[:, -2] - min(results[:, -2])) / (max(results[:, -2]) -   min(results[:, -2]))
norm_avg_ori_err = (results[:, -1] - min(results[:, -1])) / (max(results[:, -1]) -   min(results[:, -1]))
combined_err = np.sqrt(np.square(norm_avg_loc_err) + np.square(norm_avg_ori_err)) 

comb = np.zeros((np.size(results,0), np.size(results,1)+3))
comb[:,0:np.size(results,1)] = results
comb[:,-3] = norm_avg_loc_err
comb[:,-2] = norm_avg_ori_err
comb[:,-1] = combined_err

results = comb

perc_val = np.percentile(results[:, perc_data_ind],perc_p)
results = results[results[:, perc_data_ind] < perc_val]


#####################################
# 3d plot
#####################################

points = go.Scatter3d(
    mode = "markers",
    z=results[:,z_ind],
    x=results[:,x_ind],
    y=results[:,y_ind],
    hoverinfo = "text",
    text=results[:,perc_data_ind],
    marker=dict(
        cmin=min(results[:,perc_data_ind]),
        cmax=max(results[:,perc_data_ind]),
        color=results[:,perc_data_ind],
        colorscale = [[0, 'rgb(255,0,0)'], [1, 'rgb(0,0,255)']]
    )
)
    
layout = go.Layout(
    title = "{}% Percentile Simulation Results, SEM {} ".format(perc_p, sem),
    scene = dict(                    
       xaxis = dict(title = headers[x_ind]),
       yaxis = dict(title = headers[y_ind]),
       zaxis = dict(title = headers[z_ind]),
    )
)

fig = dict( data=[points], layout=layout)

offline.plot(fig, filename='plot3d.html')

#####################################
# 2d aggregation
#####################################

unique_y = np.sort(np.unique(results[:,y_ind]))
unique_z = np.sort(np.unique(results[:,z_ind]))

results_xavg = np.zeros((len(unique_y) * len(unique_z), 5))

for i in range(len(results)):
    z = np.searchsorted(unique_z, results[i, z_ind])
    y = np.searchsorted(unique_y, results[i, y_ind])
    
    results_xavg[y + z* len(unique_y), 0] = results[i, z_ind]
    results_xavg[y + z* len(unique_y), 1] = results[i, y_ind]
    results_xavg[y + z* len(unique_y), 2] = results_xavg[y + z* len(unique_y), 2] + results[i, perc_data_ind]
    results_xavg[y + z* len(unique_y), 3] = results_xavg[y + z* len(unique_y), 3] + 1

results_xavg[:,4] = results_xavg[:,2] /  results_xavg[:,3]


#####################################
# 2d aggregated plot
#####################################

xavg_hm = go.Heatmap(
    z=results_xavg[:,4] ,
    x=results_xavg[:,0],
    y=results_xavg[:,1],
)

fig = dict( data=[xavg_hm])
offline.plot(fig, filename='plot2d.html')