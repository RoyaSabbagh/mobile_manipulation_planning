#!/usr/bin/env python
# license removed for brevity
import rospy
import numpy as np
import copy
import csv
import matplotlib
import matplotlib.pyplot as plt
from pylab import plot, show, savefig, xlim, figure, \
                hold, ylim, legend, boxplot, setp, axes

def setBoxColors(bp, edge_color, fill_color, mid_color):
		for element in ['boxes', 'whiskers', 'fliers', 'caps']:
			plt.setp(bp[element], color=edge_color)
		plt.setp(bp['medians'], color=mid_color)
		for patch in bp['boxes']:
			patch.set(facecolor=fill_color)

def read_csv(num_experiment, method):
	success = 0
	error_pos = []
	error_ori = []
	time_sim = 0
	time_run = 0
    dirname = os.path.dirname(__file__)
    filename = os.path.join(dirname, 'result/sim_{}_experiment{}_final.csv'.format(method, num_experiment))
	with open(filename) as file:
				readCSV = csv.reader(file)
				headers = next(readCSV)
				for row in readCSV:
					success += float(row[0])
					time_sim += float(row[1])
					time_run += float(row[2])
					error_pos.append(float(row[3]))
					error_ori.append(float(row[4])/180*np.pi)
				success_rate = float(success)/ (len(error_pos))
				ave_time_sim = float(time_sim)/ (len(error_pos))
				ave_time_run = float(time_run)/ (len(error_pos))

	return [success_rate, ave_time_sim, ave_time_run, error_pos, error_ori]

experiment_num = 4
num_obj = 4
blue_circle = dict(markerfacecolor='lightseagreen', marker='o')
red_circle = dict(markerfacecolor='peru', marker='o')
N = experiment_num
ind = np.arange(N)
width = 0.3
fig,ax = plt.subplots(2,4, figsize=(15, 15))
bp1 = [[],[],[],[]]
bp2 = [[],[],[],[]]
bp3 = [[],[],[],[]]
bp4 = [[],[],[],[]]
bp5 = [[],[],[],[]]
bp6 = [[],[],[],[]]
bp7 = [[],[],[],[]]
bp8 = [[],[],[],[]]

bp9 = [[],[],[],[]]
bp10 = [[],[],[],[]]
bp11 = [[],[],[],[]]
bp12 = [[],[],[],[]]
bp13 = [[],[],[],[]]
bp14 = [[],[],[],[]]
bp15 = [[],[],[],[]]
bp16 = [[],[],[],[]]

for obj in range(num_obj):
    pos_LQR = []
    pos_opt = []
    ori_LQR = []
    ori_opt = []
    suc_rate_opt =[]
    suc_rate_LQR =[]
    for num in range(obj*4,obj*4+experiment_num):
    	success_rate, ave_time_sim, ave_time_run, error_pos, error_ori = read_csv(num+1, "opt")
    	success_rate_LQR, ave_time_sim_LQR, ave_time_run_LQR, error_pos_LQR, error_ori_LQR = read_csv(num+1, "LQR")
    	print "success rate for experiment {} with optimization method is %{}.".format(obj*4+num+1, success_rate*100)
    	print "avarage simulation time for experiment {} with optimization method is {}s.".format(obj*4+num+1, ave_time_sim)
    	print "avarage running time for experiment {} with optimization method is {}s.".format(obj*4+num+1, ave_time_run)
    	suc_rate_opt.append(success_rate)
    	pos_opt.append(error_pos)
    	ori_opt.append(error_ori)
    	print "success rate for experiment {} with LQR method is %{}.".format(obj*4+num+1, success_rate_LQR*100)
    	print "avarage simulation time for experiment {} with LQR method is {}s.".format(obj*4+num+1, ave_time_sim_LQR)
    	print "avarage running time for experiment {} with LQR method is {}s.".format(obj*4+num+1, ave_time_run_LQR)
        print "*******************************************************************************"
    	suc_rate_LQR.append(success_rate)
    	pos_LQR.append(error_pos_LQR)
    	ori_LQR.append(error_ori_LQR)


    bp1[obj] = ax[0,obj].boxplot(pos_opt[0], positions = [1-0.2], widths = width, patch_artist=True, flierprops=blue_circle)
    setBoxColors(bp1[obj], 'black', 'lightseagreen', 'black')
    bp2[obj] = ax[0,obj].boxplot(pos_LQR[0], positions = [1+0.2], widths = width, patch_artist=True, flierprops=red_circle)
    setBoxColors(bp2[obj], 'black', 'peru', 'black')

    bp3[obj] = ax[0,obj].boxplot(pos_opt[1], positions = [2-0.2], widths = width, patch_artist=True, flierprops=blue_circle)
    setBoxColors(bp3[obj], 'black', 'lightseagreen', 'black')
    bp4[obj] = ax[0,obj].boxplot(pos_LQR[1], positions = [2+0.2], widths = width, patch_artist=True, flierprops=red_circle)
    setBoxColors(bp4[obj], 'black', 'peru', 'black')

    bp5[obj] = ax[0,obj].boxplot(pos_opt[2], positions = [3-0.2], widths = width, patch_artist=True, flierprops=blue_circle)
    setBoxColors(bp5[obj], 'black', 'lightseagreen', 'black')
    bp6[obj] = ax[0,obj].boxplot(pos_LQR[2], positions = [3+0.2], widths = width, patch_artist=True, flierprops=red_circle)
    setBoxColors(bp6[obj], 'black', 'peru', 'black')

    bp7[obj] = ax[0,obj].boxplot(pos_opt[3], positions = [4-0.2], widths = width, patch_artist=True, flierprops=blue_circle)
    setBoxColors(bp7[obj], 'black', 'lightseagreen', 'black')
    bp8[obj] = ax[0,obj].boxplot(pos_LQR[3], positions = [4+0.2], widths = width, patch_artist=True, flierprops=red_circle)
    setBoxColors(bp8[obj], 'black', 'peru', 'black')

    ax[0,obj].set_ylim((0,3))
    ax[0,obj].set_xlim((0.5,4.5))


    ax[0,obj].axhline(0.1, color="red", linestyle ='--', linewidth=2)
    if obj == 0:
        ax[0,obj].text(-0.6, 0.08, '$\mathrm{Success}$', fontsize=14)
        ax[0,obj].set_ylabel('$\mathrm{Position \; Error \; (m)}$', fontsize=22)
        ax[0,obj].text(1.85, 3.1, '$\mathrm{Walker}$', fontsize=18)
    if obj == 1:
        ax[0,obj].text(1.6, 3.1, '$\mathrm{Blue \; chair}$', fontsize=18)
    if obj == 2:
        ax[0,obj].text(1.5, 3.1, '$\mathrm{Gray \; chair}$', fontsize=18)
    if obj == 3:
        ax[0,obj].text(2, 3.1, '$\mathrm{Rack}$', fontsize=18)
    ax[0,obj].set_xticklabels(['$\mathrm{1}$', '$\mathrm{2}$', '$\mathrm{3}$', '$\mathrm{4}$'], fontsize=16)
    ax[0,obj].set_yticklabels(['$\mathrm{0.0}$', '$\mathrm{0.5}$', '$\mathrm{1.0}$', '$\mathrm{1.5}$', '$\mathrm{2.0}$', '$\mathrm{2.5}$'], fontsize=16)

    ax[0,obj].set_xticks([1, 2 , 3 , 4])
    ax[0,obj].set_yticks([0, 0.5, 1 , 1.5 , 2, 2.5])

    if obj == 3:
        ax[0,obj].legend([bp1[obj]["boxes"][0], bp2[obj]["boxes"][0]], ['$\mathrm{Optimization}$', '$\mathrm{LQR}$'], loc='upper center', ncol=2, bbox_to_anchor=(0.27, 1.22), fontsize=16)


    # plt.savefig("/home/roya/experiment_sim_pos_{}.pdf".format(obj), dpi =300)
    ax[0,obj].yaxis.grid(True)


    #******************************************************************************************
    bp9[obj] = ax[1,obj].boxplot(ori_opt[0], positions = [1-0.2], widths = width, patch_artist=True, flierprops=blue_circle)
    setBoxColors(bp9[obj], 'black', 'lightseagreen', 'black')
    bp10[obj] = ax[1,obj].boxplot(ori_LQR[0], positions = [1+0.2], widths = width, patch_artist=True, flierprops=red_circle)
    setBoxColors(bp10[obj], 'black', 'peru', 'black')

    bp11[obj] = ax[1,obj].boxplot(ori_opt[1], positions = [2-0.2], widths = width, patch_artist=True, flierprops=blue_circle)
    setBoxColors(bp11[obj], 'black', 'lightseagreen', 'black')
    bp12[obj] = ax[1,obj].boxplot(ori_LQR[1], positions = [2+0.2], widths = width, patch_artist=True, flierprops=red_circle)
    setBoxColors(bp12[obj], 'black', 'peru', 'black')

    bp13[obj] = ax[1,obj].boxplot(ori_opt[2], positions = [3-0.2], widths = width, patch_artist=True, flierprops=blue_circle)
    setBoxColors(bp13[obj], 'black', 'lightseagreen', 'black')
    bp14[obj] = ax[1,obj].boxplot(ori_LQR[2], positions = [3+0.2], widths = width, patch_artist=True, flierprops=red_circle)
    setBoxColors(bp14[obj], 'black', 'peru', 'black')

    bp15[obj] = ax[1,obj].boxplot(ori_opt[3], positions = [4-0.2], widths = width, patch_artist=True, flierprops=blue_circle)
    setBoxColors(bp15[obj], 'black', 'lightseagreen', 'black')
    bp16[obj] = ax[1,obj].boxplot(ori_LQR[3], positions = [4+0.2], widths = width, patch_artist=True, flierprops=red_circle)
    setBoxColors(bp16[obj], 'black', 'peru', 'black')

    ax[1,obj].set_ylim((0,3))
    ax[1,obj].set_xlim((0.5,4.5))


    ax[1,obj].axhline(0.2, color="red", linestyle ='--', linewidth=2)
    if obj == 0:
        ax[1,obj].text(-0.6, 0.17, '$\mathrm{Success}$', fontsize=14)
        ax[1,obj].set_ylabel('$\mathrm{Orientation \; Error \; (rad)}$', fontsize=22)
    ax[1,obj].set_xticklabels(['$\mathrm{1}$', '$\mathrm{2}$', '$\mathrm{3}$', '$\mathrm{4}$'], fontsize=16)
    ax[1,obj].set_yticklabels(['$\mathrm{0.0}$', '$\mathrm{0.5}$', '$\mathrm{1.0}$', '$\mathrm{1.5}$', '$\mathrm{2.0}$', '$\mathrm{2.5}$'], fontsize=16)

    ax[1,obj].set_xticks([1, 2 , 3 , 4])
    ax[1,obj].set_yticks([0, 0.5, 1 , 1.5 , 2, 2.5])
    ax[1,obj].yaxis.grid(True)

dirname = os.path.dirname(__file__)
filename = os.path.join(dirname, "results/experiment_sim.pdf")
plt.savefig(filename, dpi =300)
plt.show()
