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
import codecs


def setBoxColors(bp, edge_color, fill_color, mid_color):
		for element in ['boxes', 'whiskers', 'fliers', 'caps']:
			plt.setp(bp[element], color=edge_color)
		plt.setp(bp['medians'], color=mid_color)
		for patch in bp['boxes']:
			patch.set(facecolor=fill_color)

def read_csv(num_experiment):
	success = 0
	error_pos = []
	error_ori = []
	time_sim = 0
	time_run = 0
    dirname = os.path.dirname(__file__)
    filename = os.path.join(dirname, 'result/real_experiment{}_final.csv'.format(num_experiment))
	readCSV = csv.reader(codecs.open(filename, 'rU', 'utf-16'))
	headers = next(readCSV)
	for row in readCSV:
		success += float(row[0])
		time_run += float(row[1])
		error_pos.append(float(row[2]))
		error_ori.append(float(row[3])/180*np.pi)
	success_rate = float(success)/ (len(error_pos))
	ave_time_run = float(time_run)/ (len(error_pos))

	return [success_rate, ave_time_run, error_pos, error_ori]

experiment_num = 2
pos_opt = []
ori_opt = []
suc_rate_opt =[]
for num in range(experiment_num):
	success_rate, ave_time_run, error_pos, error_ori = read_csv(num+1)
	print "success rate for experiment {} with optimization method is %{}.".format(num+1, success_rate*100)
	print "avarage running time for experiment {} with optimization method is {}s.".format(num, ave_time_run)
	suc_rate_opt.append(success_rate)
	pos_opt.append(error_pos)
	ori_opt.append(error_ori)

blue_circle = dict(markerfacecolor='b', marker='o')
red_circle = dict(markerfacecolor='r', marker='o')
N = experiment_num
ind = np.arange(N)
width = 0.3
fig,ax = plt.subplots(1,2, figsize=(15, 15))

bp1 = ax[0].boxplot(pos_opt[0], positions = [0.8], widths = width, patch_artist=True, flierprops=blue_circle)
setBoxColors(bp1, 'black', 'lightseagreen', 'black')

bp2 = ax[0].boxplot(pos_opt[1], positions = [1.4], widths = width, patch_artist=True, flierprops=blue_circle)
setBoxColors(bp2, 'black', 'lightseagreen', 'black')

ax[0].set_ylim((0,3))
ax[0].set_xlim((0.5,2.1))

ax[0].text(1.7, 0.25, '$\mathrm{Success}$', fontsize=28)
ax[0].text(0.6, 3.1, '$\mathrm{Position \; Error (m)}$', fontsize=42)
ax[0].axhline(0.2, color="red", linestyle ='--', linewidth=2)
ax[0].set_xticklabels(['$\mathrm{1}$', '$\mathrm{2}$'], fontsize=32)
ax[0].set_yticklabels(['$\mathrm{0.0}$', '$\mathrm{0.5}$', '$\mathrm{1.0}$', '$\mathrm{1.5}$', '$\mathrm{2.0}$', '$\mathrm{2.5}$'], fontsize=32)
ax[0].set_xticks([0.8, 1.4])
ax[0].set_yticks([0, 0.5, 1 , 1.5 , 2, 2.5])

ax[0].yaxis.grid(True)


bp1 = ax[1].boxplot(ori_opt[0], positions = [0.8], widths = width, patch_artist=True, flierprops=blue_circle)
setBoxColors(bp1, 'black', 'lightseagreen', 'black')

bp2 = ax[1].boxplot(ori_opt[1], positions = [1.4], widths = width, patch_artist=True, flierprops=blue_circle)
setBoxColors(bp2, 'black', 'lightseagreen', 'black')

ax[1].set_ylim((0,3))
ax[1].set_xlim((0.5,2.1))

ax[1].text(1.7, 0.35, '$\mathrm{Success}$', fontsize=28)
ax[1].axhline(0.3, color="red", linestyle ='--', linewidth=2)
ax[1].text(0.4, 3.1, '$\mathrm{Orientation \; Error (rad)}$', fontsize=42)
ax[1].set_xticklabels(['$\mathrm{1}$', '$\mathrm{2}$'], fontsize=32)
ax[1].set_yticklabels(['$\mathrm{0.0}$', '$\mathrm{0.5}$', '$\mathrm{1.0}$', '$\mathrm{1.5}$', '$\mathrm{2.0}$', '$\mathrm{2.5}$'], fontsize=32)

ax[1].set_xticks([0.8, 1.4])
ax[1].set_yticks([0, 0.5, 1 , 1.5 , 2, 2.5])
ax[1].yaxis.grid(True)

dirname = os.path.dirname(__file__)
filename = os.path.join(dirname, "results/experiment_real.pdf")
plt.savefig(filename, dpi =300)

plt.show()
