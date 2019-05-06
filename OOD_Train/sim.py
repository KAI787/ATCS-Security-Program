from block import Block
from system import System
from train import Train
import matplotlib.pyplot as plt
import matplotlib.dates as mdates
from datetime import datetime, timedelta
import numpy as np
  
def string_diagram(sys, start_time, end_time):
    '''To draw the string diagram based on the schedule dictionary for all the trains. 
    '''
    colors = ['red','green','blue','black','orange','cyan','magenta']
    color_num = len(colors)
    x = []; y = []; 
    
    # for i in range(len(sys.up_trains)-1):
    #     x.append([])
    #     y.append([])
    #     for j in range(len(sys.up_trains[i].time_pos_list)-1):
    #         x[i].append(datetime.fromtimestamp(sys.up_trains[i].time_pos_list[j][0]))
    #         y[i].append(sys.up_trains[i].time_pos_list[j][1])
    #         # x[i].append(sys.up_trains[i].time_pos_list[j][0])
    #         # y[i].append(sys.up_trains[i].time_pos_list[j][1])
    print(len(sys.down_trains))
    for i in range(len(sys.down_trains)-1):
        x.append([])
        y.append([])
        for j in range(len(sys.down_trains[i].time_pos_list)-1):
            x[i].append(datetime.fromtimestamp(sys.down_trains[i].time_pos_list[j][0]))
            y[i].append(sys.down_trains[i].time_pos_list[j][1])
            # x[i].append(sys.down_trains[i].time_pos_list[j][0])
            # y[i].append(sys.down_trains[i].time_pos_list[j][1])
    y = [i for _,i in sorted(zip([i[0] for i in x], y))]
    x = sorted(x, key = lambda x: x[0])
    assert len(x) == len(y)
    
    train_idx = list(range(len(x)))
    t_color = [colors[x.index(i)%color_num] for i in x]
    min_t, max_t = min([i[0] for i in x]), max([i[-1] for i in x])

    #plt.ion()
    plt.title('Result Analysis')
    hours = mdates.HourLocator()
    minutes = mdates.MinuteLocator()
    seconds = mdates.SecondLocator()
    dateFmt = mdates.DateFormatter("%H:%M")
    plt.gca().xaxis.set_major_locator(hours)
    plt.gca().xaxis.set_minor_locator(minutes)
    plt.gca().xaxis.set_major_formatter(dateFmt)
    plt.xticks(rotation=90)
    plt.grid(True, linestyle = "-.", color = "r", linewidth = "0.1")
    plt.legend()
    plt.xlabel('Time')
    plt.ylabel('Mile Post/miles')
    
#===============================================================================
# 
#     start_time = int(start_time.timestamp())
#     end_time = int(end_time.timestamp())
#     time_length = end_time - start_time
#     step_size = 10
#     for start in range(1,time_length + 1, step_size):
#         plt.axis([(datetime.fromtimestamp(start_time - 500)), \
#             (datetime.fromtimestamp(end_time + 500)), -5 , 55])
#         
#         for n in range(len(x)-1):
#             new_x_y = [[mdates.date2num(datetime.fromtimestamp(i)), j] for i, j in zip(x[n], y[n]) if i < start_time + start and i > start_time + start - 1 - step_size]
#             new_x = []
#             new_y = []
#             for i , j in new_x_y:
#                 new_x.append(i)
#                 new_y.append(j)
#             if(len(new_x) == 0):
#                 continue
#             plt.plot(new_x, new_y, color=t_color[n])
#             
#             print('==============')
#             print('Length of new_x: {}'.format(len(new_x)))
#             print('Length of new_y: {}'.format(len(new_y)))
#         plt.pause(0.00001)
#===============================================================================
    for n in range(len(x)-1):
        #assert len(x[n]) == len(y[n]) == t_color[n]
        plt.plot([mdates.date2num(i) for i in x[n]], y[n], color=t_color[n])
    plt.gca().axhspan(15,20,color='yellow',alpha=0.5)
    plt.gca().axhspan(30,35,color='yellow',alpha=0.5)
    plt.show()
    #plt.ioff()


def main():
    sim_init_time = datetime.strptime('2018-01-10 10:00:00', "%Y-%m-%d %H:%M:%S")
    sim_term_time = datetime.strptime('2018-01-10 11:30:00', "%Y-%m-%d %H:%M:%S")
    sys = System(sim_init_time, [5] * 10, [1,1,1,2,1,1,2,1,1,1], dos_period=['2018-01-01 00:30:00', '2018-01-01 01:30:00'], dos_pos=-1)
    sim_timedelta = sim_term_time - sim_init_time
    i = 0
    while (datetime.fromtimestamp(sys.sys_time) - sim_init_time).total_seconds() < sim_timedelta.total_seconds():
        i += 1
        sys.refresh()
    string_diagram(sys, sim_init_time, sim_term_time)

main()