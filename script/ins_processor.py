#! /home/rdaneel/anaconda3/lib/python3.8
# -*- coding: UTF-8 -*-

from cProfile import label
import enum
from linecache import getline
import os
import argparse
from typing import Dict, List, Tuple
import pprint
from sqlalchemy import false, true
import yaml
import matplotlib.pyplot as plt
import util
import numpy as np

class InsProcessor:
    def __init__(self, in_config) -> None:
        self.config: Dict = dict()
        config_dir = os.path.join(os.path.dirname(os.path.realpath(__file__)), in_config)
        with open(config_dir, 'r') as fin:
            self.config = yaml.load(fin, Loader=yaml.FullLoader)

        # Plot parameters
        self.max_x_num = 5  # on the x-axis
        self.fig_size:Tuple[int,int] = (17, 8)
        self.marker_size:int = 15
        self.line_width:float = 1.8
        self.text_size:int = 25
        self.fig_axs:Dict[int, Tuple[int,int]] = {1: (1,1),
                                                  2: (1,2),
                                                  3: (1,3),
                                                  4: (2,2),
                                                  5: (1,5),
                                                  6: (2,3),
                                                  8: (2,4),
                                                  9: (3,3)}
        self.y_labels:Dict[str, str] = {'br_max_ma_size': 'MA size'}
        self.x_labels:Dict[str,str] = {'iter': 'Iterations'}


    def plot_fig(self, x_index, y_index):
        pbs1 = dict()
        # with open('/home/rdaneel/PBS/iteration_data_maze16_ag60_PBS1.txt', 'r') as fin:
        with open('/home/rdaneel/PBS/iteration_data_den3_PBS0.txt', 'r') as fin:
            for line in fin:
                tmp_list = line.split(',')
                pbs1[tmp_list[0]] = list()
                for i in range(1, len(tmp_list)-1):
                    pbs1[tmp_list[0]].append(int(tmp_list[i]))

        pbs2 = dict()
        # with open('/home/rdaneel/PBS/iteration_data_maze16_ag60_PBS2.txt', 'r') as fin:
        with open('/home/rdaneel/PBS/iteration_data_den3_PBS2.txt', 'r') as fin:
            for line in fin:
                tmp_list = line.split(',')
                pbs2[tmp_list[0]] = list()
                for i in range(1, len(tmp_list)-1):
                    pbs2[tmp_list[0]].append(int(tmp_list[i]))
        
        max_len = max(len(pbs1[y_index]), len(pbs2[y_index]))
        if len(pbs1[y_index]) < max_len:
            for _ in range(max_len-len(pbs1[y_index])):
                pbs1[y_index].append(np.inf)
        if len(pbs2[y_index]) < max_len:
            for _ in range(max_len-len(pbs2[y_index])):
                pbs2[y_index].append(np.inf)


        fig, ax = plt.subplots()
        ax.plot(range(1, max_len+1), pbs1[y_index], label='PBS(Ori)',
                 marker='P', color='grey', markerfacecolor='white', markeredgewidth=self.line_width,
                 linewidth=self.line_width, ms=self.marker_size-5)
        ax.plot(range(1, max_len+1), pbs2[y_index], label='PBS(MCS)',
                 marker='s', color='green', markerfacecolor='white', markeredgewidth=self.line_width,
                 linewidth=self.line_width, ms=self.marker_size-5)

        y_list = list(range(0, 21, 5))
        ax.axes.set_yticks(y_list)
        ax.axes.set_yticklabels(y_list, fontsize=self.text_size)
        
        for tmp_l in ax.get_xticklabels():
            tmp_l.set_fontsize(self.text_size)

        plt.show()
        

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Take config.yaml as input!')
    parser.add_argument('--config', type=str, default='config.yaml')

    args = parser.parse_args()

    # Create instance processor
    ins_processor = InsProcessor(args.config)
    ins_processor.plot_fig(x_index='iter', y_index='br_max_ma_size')
