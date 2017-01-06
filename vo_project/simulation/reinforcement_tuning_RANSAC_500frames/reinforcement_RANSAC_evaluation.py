#####################################
# imports
# PREQUISITES: install plot.ly
#####################################
import plotly.graph_objs as go
import plotly.offline as offline
from plotly import tools
import numpy as np


#####################################
# load data
#####################################

overview = np.genfromtxt('sim_overview.csv', delimiter=',')

with open('sim_overview_h.txt') as header:
    text = header.readline()
    headers = text.split(',')

#####################################
# config
#####################################


#advanced config: colum <-> axis mapping
p0_i = 1
p1_i = 2
p2_i = 3
p3_i = 4
p4_i = 5

p_arr_i=[p0_i, p1_i ,p2_i ,p3_i ,p4_i ]

p0_name = headers[p0_i]
p1_name = headers[p1_i]
p2_name = headers[p2_i]
p3_name = headers[p3_i]
p4_name = headers[p4_i]

p_arr_n=[p0_name, p1_name ,p2_name ,p3_name ,p4_name ]
param_scale = ['category','category','category','category','category' ]

loc_err_i = -2
ori_err_i = -1

num_params = 5
bestN = 200

#####################################
# data preparation
#####################################


    
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
        
exceptions = results [results[:,-3] == 2] #failure
results = results [results[:,-3]== 1] #success
 
bestN_loc_err = results[results[:,loc_err_i].argsort()][0:bestN]
bestN_ori_err = results[results[:,ori_err_i].argsort()][0:bestN]     

data =  bestN_loc_err                       
                        
                        
#####################################
# data preparation
#####################################                  

plots = np.empty((num_params, num_params), dtype=object)
          
for i in range(num_params):
    for j in range(num_params):
    
        plots[i,j] = go.Histogram2d(
            x = np.sort(data[:,p_arr_i[i]]),
            y = np.sort(data[:,p_arr_i[j]]),
        )

#        plots[i,j] = go.Scatter(
#            x = data[:,p_arr_i[i]],
#            y = data[:,p_arr_i[j]],
#            mode = 'markers',
#            marker = dict(color = data[:,loc_err_i])
#        )


fig = tools.make_subplots(rows=num_params, cols=num_params)
    

for i in range(num_params):
    for j in range(num_params):
        fig.append_trace(plots[i,j], i+1, j+1)
        
        
for i in range(num_params):
    for j in range(num_params):
        plotNumStr = str((i+num_params*j)+1)
        xaxis = fig['layout']['xaxis{}'.format(plotNumStr)]
        yaxis = fig['layout']['yaxis{}'.format(plotNumStr)]

        xaxis.update(title=p_arr_n[j])
        yaxis.update(title=p_arr_n[i])
        
        xaxis.update(type=param_scale[j])
        yaxis.update(type=param_scale[i])
        
        xaxis.update(categoryorder='trace')
        yaxis.update(categoryorder='trace')
    
offline.plot(fig, filename="params_scatter.html", auto_open=False)          



                        
#####################################
# data preparation
#####################################     


plots = np.empty((num_params), dtype=object)


for i in range(num_params):
    plots[i] = go.Histogram(
        x=np.sort(data[:,p_arr_i[i]]),
    )

fig = tools.make_subplots(rows=1, cols=num_params)

for i in range(num_params):
    fig.append_trace(plots[i], 1, i+1)
    
for i in range(num_params):
    plotNumStr = str(i+1)
    xaxis = fig['layout']['xaxis{}'.format(plotNumStr)]
    xaxis.update(title=p_arr_n[i])
    xaxis.update(type=param_scale[i])
    xaxis['titlefont'].update(size=20)

offline.plot(fig, filename="param_histograms.html", auto_open=False)    
              

#####################################
# histograms
#####################################  
plots = np.empty((2), dtype=object)

plots[0] = go.Histogram(
        x=results[results[:,loc_err_i].argsort()][:,loc_err_i]
    )

plots[1] = go.Histogram(
        x=results[results[:,ori_err_i].argsort()][0:-1,ori_err_i]
    )

fig = tools.make_subplots(rows=1, cols=2)

fig.append_trace(plots[0], 1, 1)
fig.append_trace(plots[1], 1, 2)

fig['layout']['xaxis1'].update(title="average location error")
fig['layout']['xaxis2'].update(title="average orientation error")

fig['layout']['yaxis1'].update(title="# occurences")
fig['layout']['yaxis2'].update(title="# occurences")

fig['layout']['yaxis1'].update(title="# occurences")
fig['layout']['yaxis2'].update(title="# occurences")


fig['layout']['xaxis1']['titlefont'].update(size=25)
fig['layout']['xaxis2']['titlefont'].update(size=25)
fig['layout']['yaxis1']['titlefont'].update(size=25)
fig['layout']['yaxis2']['titlefont'].update(size=25)

offline.plot(fig, filename="error_histograms.html", auto_open=False)    
