#####################################
# imports
# PREQUISITES: install plot.ly
#####################################
import plotly.graph_objs as go
import plotly.offline as offline
from plotly import tools
import numpy as np

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
    if(int(overview[i,3]) == 1 ):
        errors = np.genfromtxt('task{}_errors.csv'.format(task_i), delimiter=',')
        avg_loc_error = errors[-1, 2]
        avg_ori_error = errors[-1, 3]
        
        results[i] = np.concatenate((overview[i], [avg_loc_error, avg_ori_error]), 0)
        
    elif (int(overview[i,3]) == 2 ):
        results[i] = np.concatenate((overview[i], [0, 0]), 0)
        
        
results = results [results[:,-1] > 0]

#error normalization
norm_avg_loc_err = (results[:, -2] - min(results[:, -2])) / (max(results[:, -2]) -   min(results[:, -2]))
norm_avg_ori_err = (results[:, -1] - min(results[:, -1])) / (max(results[:, -1]) -   min(results[:, -1]))
combined_err = np.sqrt(np.square(norm_avg_loc_err) + np.square(norm_avg_ori_err)) 

#####################################
# plotting
#####################################

success = go.Scatter(
    x = overview[:,1],
    y = overview[:,2],
    mode = 'markers',
    marker = dict(color = overview[:,3])
)

hm_loc_err = go.Heatmap(
    z=norm_avg_loc_err,
    x=results[:,1],
    y=results[:,2],
)

hm_ori_err = go.Heatmap(
    z=norm_avg_ori_err,  
    x=results[:,1],
    y=results[:,2],
)

eucl_norm_err = go.Heatmap(
    z=combined_err ,  
    x=results[:,1],
    y=results[:,2],
)

fig = tools.make_subplots(rows=1, cols=4, subplot_titles=('sucess',
                                                          'location error',
                                                          'orientation error',
                                                          'eucl normal. error'))
    
fig.append_trace(success, 1, 1)
fig.append_trace(hm_loc_err, 1, 2)
fig.append_trace(hm_ori_err, 1, 3)
fig.append_trace(eucl_norm_err, 1, 4)
    
offline.plot(fig, filename="score-heatmap.html")
