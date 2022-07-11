#! /home/rdaneel/anaconda3/lib/python3.8
# -*- coding: UTF-8 -*-
"""Data processor"""

import logging
import os
import sys
import argparse
from typing import Dict, List, Tuple
import yaml
import matplotlib.pyplot as plt
import util
import numpy as np

class DataProcessor:
    def __init__(self, in_config) -> None:
        self.config: Dict = dict()
        config_dir = os.path.join(os.path.dirname(os.path.realpath(__file__)), in_config)
        with open(config_dir, 'r') as fin:
            self.config = yaml.load(fin, Loader=yaml.FullLoader)

        # Plot parameters
        self.max_x_num = 5  # on the x-axis
        self.fig_size:Tuple[int,int] = (12, 9) # (17, 8)
        self.marker_size:int = 25
        self.line_width:float = 4.0
        self.mark_width:float = 4.0
        self.text_size:int = 40
        self.fig_axs:Dict[int, Tuple[int,int]] = {1: (1,1),
                                                  2: (1,2),
                                                  3: (1,3),
                                                  4: (2,2),
                                                  5: (1,5),
                                                  6: (2,3),
                                                  8: (2,4),
                                                  9: (3,3)}
        self.y_labels:Dict[str, str] = {'succ': 'Success Rate',
                                        'runtime': 'Runtime (sec)',
                                        'solution cost': 'SoC',
                                        'max_ma_size': 'Max PA Size',
                                        '#low-level generated': '# Generated LL Nodes (M)',  # (M)
                                        '#low-level expanded': '# Expanded LL nodes (M)',
                                        '#high-level generated': '# Generated HL Nodes',
                                        '#high-level expanded': '# Expanded HL nodes (K)',
                                        '#pathfinding': '# Replaned Agents', # (K)
                                        'num_in_conf': 'Internal / Total',
                                        'num_ex_conf': 'External / Total',
                                        'num_total_conf': '# Total Conflicts (K)',
                                        'num_0child': '# Backtrack (K)',
                                        'add': 'Sum (K)',
                                        'sub': 'Subtraction',
                                        'mul': 'Multiplication',
                                        'div': '# Replans',
                                        'mod': 'Mod'}
        self.x_labels:Dict[str,str] = {'num': '# Agents',
                                       'ins': 'Instance'}

    def get_subfig_pos(self, f_idx: int):
        """Transfer subplot index to 2-D position
        Args:
            f_idx (int): subplot index

        Returns:
            int, int: 2D position
        """
        f_row = self.fig_axs[len(self.config['maps'])][1]
        return f_idx // f_row, f_idx % f_row


    def get_val(self, x_index:str='num', y_index:str='succ', is_avg:bool=True):
        """Get the value on the y axid

        Args:
            x_index (str, optional): value of the x-axid. Defaults to 'num'.
            y_index (str, optional): value of the y-axid. Defaults to 'succ'.
            is_avg (bool, optional): whether to averaging the y value. Defaults to True.

        Returns:
            Dict: the y value on the y axid
        """
        if x_index == 'ins':
            return self.get_ins_val(y_index)
        elif x_index == 'num':
            return self.get_num_val(y_index, is_avg)

    def get_ins_val(self, in_index:str='runtime'):
        """Compute the success rate versus the numbers of agents

        Args:
            in_index (str, optional): which data we want to analyze. Defaults to 'runtime'.

        Returns:
            Dict: the success rate (versus the numbers of agents) of each solver
        """

        result: Dict = dict()
        for solver in self.config['solvers']:
            result[solver['name']] = dict()
            for _map_ in self.config['maps']:
                result[solver['name']][_map_['name']] = {'x': list(), 'val': list(), 'ci': list()}
                global_idx = 1

                for ag_num in _map_['num_of_agents']:
                    for scen in _map_['scens']:
                        data_frame = util.get_csv_instance(self.config['exp_path'], _map_['name'],
                                                           scen, ag_num, solver['name'])
                        for _, row in data_frame.iterrows():
                            if in_index == 'runtime':
                                tmp_val = min(row[in_index], 60)
                            elif in_index == 'num_in_conf':
                                if row['num_total_conf'] == 0 or row['num_in_conf'] == 0:
                                    tmp_val = np.inf
                                else:
                                    tmp_val = row['num_in_conf']/row['num_total_conf']
                            elif in_index == 'num_ex_conf':
                                if row['num_total_conf'] == 0 or row['num_ex_conf'] == 0:
                                    tmp_val = np.inf
                                else:
                                    tmp_val = row['num_ex_conf']/row['num_total_conf']
                            elif in_index == 'num_total_conf':
                                if row['num_total_conf'] == 0:
                                    tmp_val = np.inf
                                else:
                                    tmp_val = row['num_total_conf']
                            elif in_index == 'num_0child':
                                if row['num_0child'] == 0:
                                    tmp_val = np.inf
                                else:
                                    tmp_val = row['num_0child']
                            elif row[in_index] < 0:
                                tmp_val = np.inf
                            else:
                                tmp_val = row[in_index]

                            result[solver['name']][_map_['name']]['val'].append(tmp_val)
                            result[solver['name']][_map_['name']]['x'].append(global_idx)
                            global_idx += 1
        return result


    def get_num_val(self, in_index:str='succ', is_avg:bool=True):
        """Compute the success rate versus the numbers of agents

        Args:
            in_index (str, optional): which data we want to analyze. Defaults to 'succ'.

        Returns:
            Dict: the success rate (versus the numbers of agents) of each solver
        """

        result: Dict = dict()
        for solver in self.config['solvers']:
            result[solver['name']] = dict()
            for _map_ in self.config['maps']:
                result[solver['name']][_map_['name']] = {'x': list(), 'val': list(), 'ci': list()}

                for ag_num in _map_['num_of_agents']:
                    total_val = 0.0
                    total_num = 0.0
                    _data_:List = list()
                    for scen in _map_['scens']:
                        tmp_ins_num = 0
                        data_frame = util.get_csv_instance(self.config['exp_path'], _map_['name'],
                                                           scen, ag_num, solver['name'])
                        for _, row in data_frame.iterrows():
                            tmp_ins_num += 1
                            if in_index == 'succ':
                                if row['solution cost'] >= 0 and \
                                    row['runtime'] <= self.config['time_limit']:
                                    total_val += 1
                            elif in_index == 'runtime':
                                _data_.append(min(row[in_index], 60))
                                total_val += min(row[in_index], 60)
                            elif in_index == '#high-level generated':
                                assert row[in_index] > 0
                                if row[in_index] == 1 and '#pathfinding' in row.index:
                                    assert row['#pathfinding'] == 0
                                _data_.append(row[in_index]-1)
                                total_val += row[in_index]-1
                            else:
                                assert row[in_index] >= 0
                                _data_.append(row[in_index])
                                total_val += row[in_index]

                        total_num += self.config['ins_num']
                        if tmp_ins_num != self.config['ins_num']:
                            logging.warning('Ins number does no match at map:%s, scen:%s, ag:%d',
                                            _map_['name'], scen, ag_num)

                    if is_avg:
                        if total_num == 0:
                            _rate_ = 0
                        else:
                            _rate_ = total_val / total_num  # average value
                    else:
                        _rate_ = total_val

                    result[solver['name']][_map_['name']]['x'].append(ag_num)
                    result[solver['name']][_map_['name']]['val'].append(_rate_)

                    if self.config['plot_ci'] and len(_data_) > 0:  # non empty list
                        _ci_ = 1.96*np.std(_data_) / np.sqrt(total_num)  # confident interval
                        result[solver['name']][_map_['name']]['ci'].append(_ci_)
                    elif self.config['plot_std'] and len(_data_) > 0:
                        _ci_ = np.std(_data_)  # standard deviation
                        result[solver['name']][_map_['name']]['ci'].append(_ci_)

        return result


    def get_w_val(self, in_index:str, is_avg:bool=True):
        """Compute the success rate versus the numbers of agents

        Args:
            in_index (str, optional): which data we want to analyze.

        Returns:
            Dict: the success rate (versus the numbers of agents) of each solver
        """

        result: Dict = dict()
        for solver in self.config['solvers']:
            result[solver['name']] = dict()
            for _map_ in self.config['maps']:
                result[solver['name']][_map_['name']] = {'x': list(), 'val': list(), 'ci': list()}
                default_w = solver['w']

                for tmp_fw in self.config['f_weights']:
                    solver['w'] = tmp_fw
                    total_val = 0.0
                    total_num = 0.0
                    _data_:List = list()

                    for ag_num in _map_['num_of_agents']:
                        for scen in _map_['scens']:
                            data_frame = util.get_csv_instance(self.config['exp_path'],
                                                    _map_['name'], scen, ag_num, solver['name'])
                            for _, row in data_frame.iterrows():
                                if in_index == 'succ':
                                    if row['solution cost'] >= 0 and \
                                        row['runtime'] <= self.config['time_limit']:
                                        total_val += 1
                                elif in_index == 'runtime':
                                    _data_.append(min(row[in_index], 60))
                                    total_val += min(row[in_index], 60)
                                else:
                                    assert row[in_index] >= 0
                                    _data_.append(row[in_index])
                                    total_val += row[in_index]

                            total_num += self.config['ins_num']

                    if is_avg:
                        _rate_ = total_val / total_num  # average value
                    else:
                        _rate_ = total_val

                    result[solver['name']][_map_['name']]['x'].append(tmp_fw)
                    result[solver['name']][_map_['name']]['val'].append(_rate_)

                    if self.config['plot_ci'] and len(_data_) > 0:  # non empty list
                        # _ci_ = 1.96*np.std(_data_) / np.sqrt(total_num)  # confident interval
                        _ci_ = np.std(_data_)  # standard deviation
                        result[solver['name']][_map_['name']]['ci'].append(_ci_)

                solver['x'] = default_w

        return result


    def subplot_fig(self, x_index, y_index, in_axs, in_map_idx, in_map, in_result):
        _x_ = in_result[self.config['solvers'][0]['name']][in_map['name']]['x']
        _num_ = range(1, len(_x_)+1)

        for solver in self.config['solvers']:
            _val_ = in_result[solver['name']][in_map['name']]['val']
            _ci_  = in_result[solver['name']][in_map['name']]['ci']

            if in_map_idx == 0:
                if (self.config['plot_std'] or self.config['plot_ci']) and len(_ci_) > 0:
                    in_axs.errorbar(_num_, _val_, yerr=_ci_,
                                    label=solver['label'],
                                    linewidth=self.line_width,
                                    markerfacecolor='white',
                                    markeredgewidth=self.mark_width,
                                    ms=self.marker_size,
                                    color=solver['color'],
                                    marker=solver['marker'])
                else:
                    in_axs.plot(_num_, _val_,
                                label=solver['label'],
                                linewidth=self.line_width,
                                markerfacecolor='white',
                                markeredgewidth=self.mark_width,
                                ms=self.marker_size,
                                color=solver['color'],
                                marker=solver['marker'])
            else:
                if (self.config['plot_std'] or self.config['plot_ci']) and len(_ci_) > 0:
                    in_axs.errorbar(_num_, _val_, yerr=_ci_,
                                    linewidth=self.line_width,
                                    markerfacecolor='white',
                                    markeredgewidth=self.mark_width,
                                    ms=self.marker_size,
                                    color=solver['color'],
                                    marker=solver['marker'])
                else:
                    in_axs.plot(_num_, _val_,
                                linewidth=self.line_width,
                                markerfacecolor='white',
                                markeredgewidth=self.mark_width,
                                ms=self.marker_size,
                                color=solver['color'],
                                marker=solver['marker'])

            # # Plot confident interval with fill_between
            # if self.config['plot_ci'] and len(_ci_) > 0:
            #     _lb_ = [_val_[i] - _ci_[i] for i in range(len(_val_))]
            #     _ub_ = [_val_[i] + _ci_[i] for i in range(len(_val_))]
            #     in_axs.fill_between(_num_, _lb_, _ub_, color=solver['color'], alpha=0.2)
        if self.config['set_title']:
            in_axs.set_title(in_map['label'], fontsize=self.text_size)

        if len(_num_) > self.max_x_num and x_index == "ins":  # This is for instance analysis
            _num_ = list(range(len(_x_)//self.max_x_num, len(_x_)+1, len(_x_)//self.max_x_num))
            _num_.insert(0, 1)
            _x_ = _num_

        in_axs.axes.set_xticks(_num_)
        in_axs.axes.set_xticklabels(_x_, fontsize=self.text_size)
        in_axs.set_xlabel(self.x_labels[x_index], fontsize=self.text_size)

        y_list = in_axs.axes.get_yticks()
        if y_index == 'succ':
            y_list = [0, 0.2, 0.4, 0.6, 0.8, 1.0]
            in_axs.axes.set_yticks(y_list)

        elif y_index == 'runtime':
            y_list = range(0, 61, 10)
            in_axs.axes.set_yticks(y_list)

        elif y_index == 'max_ma_size':
            y_list = range(0, max(in_map['num_of_agents'])+5, 20)
            in_axs.axes.set_yticks(y_list)

        elif y_index == '#low-level generated':
            label_scale = 1000000
            tmp_range = 1
            scale = label_scale * tmp_range
            y_list = np.arange(0, max(y_list)+5, scale)
            in_axs.axes.set_yticks(y_list)

            if isinstance(tmp_range, float):
                y_list = [str(y/label_scale) for y in y_list]
            elif isinstance(tmp_range, int):
                y_list = [str(int(y//label_scale)) for y in y_list]

        # elif y_index == '#high-level generated':
        #     label_scale = 10
        #     tmp_range = 2
        #     scale = label_scale * tmp_range
        #     y_list = np.arange(0, max(y_list)+5, scale)

        #     in_axs.axes.set_yticks(y_list)
        #     if isinstance(tmp_range, float):
        #         y_list = [str(y/label_scale) for y in y_list]
        #     elif isinstance(tmp_range, int):
        #         y_list = [str(int(y//label_scale)) for y in y_list]

        elif y_index == '#pathfinding':
            label_scale = 1
            tmp_range = 200
            scale = label_scale * tmp_range
            y_list = np.arange(0, max(y_list)+5, scale)

            in_axs.axes.set_yticks(y_list)
            if isinstance(tmp_range, float):
                y_list = [str(y/label_scale) for y in y_list]
            elif isinstance(tmp_range, int):
                y_list = [str(int(y//label_scale)) for y in y_list]

        elif y_index == 'div':
            y_list = [0, 0.5, 1.0, 1.5]
            # y_list = [0, 2, 4, 6, 8]
            in_axs.axes.set_yticks(y_list)

        elif y_index == 'num_in_conf' or y_index == 'num_ex_conf':
            label_scale = 0.2
            scale = label_scale * 1
            # y_list = [0, 0.1, 0.2, 0.3, 0.4]
            y_list = [0, 0.2, 0.4, 0.6, 0.8, 1.0]
            # y_list = np.arange(0, max(y_list)+5, scale)
            in_axs.axes.set_yticks(y_list)
            # y_list = [str(y/label_scale) for y in y_list]

        elif y_index == 'num_total_conf':
            label_scale = 1000
            tmp_range = 10
            scale = label_scale * tmp_range
            y_list = np.arange(0, max(y_list)+5, scale)

            in_axs.axes.set_yticks(y_list)
            if isinstance(tmp_range, float):
                y_list = [str(y/label_scale) for y in y_list]
            elif isinstance(tmp_range, int):
                y_list = [str(int(y//label_scale)) for y in y_list]

        elif y_index == 'num_0child':
            label_scale = 1000
            tmp_range = 1
            scale = label_scale * tmp_range
            y_list = np.arange(0, max(y_list)+5, scale)

            in_axs.axes.set_yticks(y_list)
            if isinstance(tmp_range, float):
                y_list = [str(y/label_scale) for y in y_list]
            elif isinstance(tmp_range, int):
                y_list = [str(int(y//label_scale)) for y in y_list]

        else:
            in_axs.axes.set_yticks(y_list)

        in_axs.yaxis.grid()
        in_axs.axes.set_yticklabels(y_list, fontsize=self.text_size)
        in_axs.set_ylabel(self.y_labels[y_index], fontsize=self.text_size)

    def subplot_fig2(self, x_index, y_index, in_axs, in_result):
        _x_ = in_result[self.config['solvers'][0]['name']]['x']
        _num_ = range(1, len(_x_)+1)

        for solver in self.config['solvers']:
            _val_ = in_result[solver['name']]['val']

            in_axs.plot(_num_, _val_,
                        label=solver['label'],
                        linewidth=self.line_width,
                        markerfacecolor='white',
                        markeredgewidth=self.mark_width,
                        ms=self.marker_size,
                        color=solver['color'],
                        marker=solver['marker'])

        if len(_num_) > self.max_x_num:
            _num_ = list(range(len(_x_)//self.max_x_num, len(_x_)+1, len(_x_)//self.max_x_num))
            _num_.insert(0, 1)
            _x_ = _num_

        in_axs.axes.set_xticks(_num_)
        in_axs.axes.set_xticklabels(_x_, fontsize=self.text_size)
        in_axs.set_xlabel(self.x_labels[x_index], fontsize=self.text_size)

        y_list = in_axs.axes.get_yticks()
        if y_index == 'succ':
            y_list = [0, 0.2, 0.4, 0.6, 0.8, 1.0]
            in_axs.axes.set_yticks(y_list)
        elif y_index == 'runtime':
            y_list = range(0, 61, 10)
            in_axs.axes.set_yticks(y_list)
        in_axs.axes.set_yticklabels(y_list, fontsize=self.text_size)
        in_axs.set_ylabel(self.y_labels[y_index], fontsize=self.text_size)


    # def subplot_hist_fig(self, x_index, y_indices, in_axs, in_map_idx, in_map, in_results):
    #     _x_ = in_results[0][self.config['solvers'][0]['name']][in_map['name']]['x']
    #     _num_ = range(1, len(_x_)+1)

    #     for solver in self.config['solvers']:
    #         _val_ = in_result[solver['name']][in_map['name']]['val']
    #         _ci_  = in_result[solver['name']][in_map['name']]['ci']

    #         if in_map_idx == 0:
    #             in_axs.plot(_num_, _val_,
    #                         label=solver['label'],
    #                         linewidth=self.line_width,
    #                         markerfacecolor='white',
    #                         markeredgewidth=self.mark_width,
    #                         ms=self.marker_size,
    #                         color=solver['color'],
    #                         marker=solver['marker'])
    #         else:
    #             in_axs.plot(_num_, _val_,
    #                         linewidth=self.line_width,
    #                         markerfacecolor='white',
    #                         markeredgewidth=self.mark_width,
    #                         ms=self.marker_size,
    #                         color=solver['color'],
    #                         marker=solver['marker'])

    #         # Plot confident interval
    #         if self.config['plot_ci'] and len(_ci_) > 0:
    #             _lb_ = [_val_[i] - _ci_[i] for i in range(len(_val_))]
    #             _ub_ = [_val_[i] + _ci_[i] for i in range(len(_val_))]
    #             in_axs.fill_between(_num_, _lb_, _ub_, color=solver['color'], alpha=0.2)

    #     in_axs.set_title(in_map['label'], fontsize=self.text_size)

    #     if len(_num_) > self.max_x_num and x_index == "ins":  # This is for instance analysis
    #         _num_ = list(range(len(_x_)//self.max_x_num, len(_x_)+1, len(_x_)//self.max_x_num))
    #         _num_.insert(0, 1)
    #         _x_ = _num_

    #     in_axs.axes.set_xticks(_num_)
    #     in_axs.axes.set_xticklabels(_x_, fontsize=self.text_size)
    #     in_axs.set_xlabel(self.x_labels[x_index], fontsize=self.text_size)

    #     y_list = in_axs.axes.get_yticks()
    #     if y_index == 'succ':
    #         y_list = [0, 0.2, 0.4, 0.6, 0.8, 1.0]
    #         in_axs.axes.set_yticks(y_list)
    #     elif y_index == 'runtime':
    #         y_list = range(0, 61, 10)
    #         # y_list = range(0, 32, 5)
    #         # y_list = range(0, 26, 5)
    #         # y_list = range(0, 11, 2)
    #         # y_list = range(0, 2, 1)
    #         # y_list = [0, 0.5, 1.0, 1.5, 2.0]
    #         in_axs.axes.set_yticks(y_list)
    #     elif y_index == '#findPathForSingleAgent':
    #         in_axs.axes.set_yticks(y_list)
    #     elif y_index == '#low-level generated':
    #         label_scale = 1000000
    #         scale = label_scale * 0.1
    #         y_list = np.arange(0, max(y_list)+5, scale)
    #         # y_list = np.arange(0, max(y_list)+5, scale)
    #         # y_list = np.delete(y_list, 0)
    #         # y_list = np.delete(y_list, 0)
    #         # y_list = np.delete(y_list, -1)
    #         # y_list = np.delete(y_list, -1)
    #         in_axs.axes.set_yticks(y_list)
    #         y_list = [str(y/label_scale) for y in y_list]
    #         # y_list = [str(int(y//label_scale)) for y in y_list]
    #     elif y_index == '#high-level generated':
    #         label_scale = 1000
    #         scale = label_scale * 0.2
    #         y_list = np.arange(0, max(y_list)+5, scale)

    #         in_axs.axes.set_yticks(y_list)
    #         # y_list = [str(int(y)) for y in y_list]
    #         y_list = [str(y/label_scale) for y in y_list]
    #         # y_list = [str(int(y//label_scale)) for y in y_list]
    #     else:
    #         if y_index == 'div':
    #             y_list = [0, 0.2, 0.4, 0.6, 0.8, 1.0, 1.2]
    #             in_axs.axes.set_yticks(y_list)
    #         else:
    #             label_scale = 1000
    #             scale = label_scale * 1
    #             y_list = np.arange(0, max(y_list)+5, scale)
    #             in_axs.axes.set_yticks(y_list)
    #             y_list = [str(int(y//label_scale)) for y in y_list]        

    #     in_axs.yaxis.grid()
    #     in_axs.axes.set_yticklabels(y_list, fontsize=self.text_size)
    #     in_axs.set_ylabel(self.y_labels[y_index], fontsize=self.text_size)

    def plot_fig(self, x_index:str='num', y_index:str='succ'):
        tmp_lw = self.line_width
        if x_index == 'ins':
            self.line_width = 0.0
        # Get the result from the experiments
        result = self.get_val(x_index, y_index)

        # Plot all the subplots on the figure
        fig, axs = plt.subplots(nrows=self.fig_axs[len(self.config['maps'])][0],
                                ncols=self.fig_axs[len(self.config['maps'])][1],
                                figsize=self.fig_size,
                                dpi=80, facecolor='w', edgecolor='k')

        for idx, _map_ in enumerate(self.config['maps']):
            frow, fcol = self.get_subfig_pos(idx)
            if len(self.config['maps']) == 1:
                self.subplot_fig(x_index, y_index, axs, idx, _map_, result)
            elif self.fig_axs[len(self.config['maps'])][0] == 1:
                self.subplot_fig(x_index, y_index, axs[fcol], idx, _map_, result)
            else:
                self.subplot_fig(x_index, y_index, axs[frow,fcol], idx, _map_, result)

        fig.tight_layout()
        if self.config['set_legend']:
            if y_index == 'succ':
                plt.legend(loc="lower left", fontsize=self.text_size)
            elif y_index == 'runtime' or y_index == '#low-level generated' or \
                y_index == '#high-level generated':
                plt.legend(loc="upper left", fontsize=self.text_size)
            else:
                plt.legend(loc="best", fontsize=self.text_size)

        fig_name = ''  # Set the figure name
        for _map_ in self.config['maps']:
            fig_name += _map_['label'] + '_'
        fig_name += x_index + '_' + y_index + '_plot.png'
        plt.savefig(fig_name)
        if x_index == 'ins':
            self.line_width = tmp_lw  # set the line width back

        plt.show()

    def plot_op(self, x_index:str='num', y_index1:str='#pathfinding', 
                y_index2:str='#high-level generated',use_op:str='add'):
        """Plot the ratio between the sum of y_index1 / the sum of the y_index2

        Args:
            y_index1 (str, optional): list of the 1st numbers. Defaults to '#pathfinding'.
            y_index2 (str, optional): list of the 2nd numbers. Defaults to '#high-level generated'.
            use_op (str, optional): which operator to use. Defaults to 'div'.
        """
        op_list = ['add', 'sub', 'mul', 'div', 'mod']
        if use_op not in op_list:
            logging.error('use_op is undefined!, Should be one of the {0}'.format(op_list))
            sys.exit()

        # Get the result (sum) from the experiments
        val1 = self.get_val(x_index, y_index1, False)
        val2 = self.get_val(x_index, y_index2, False)
        x_list = val1[self.config['solvers'][0]['name']][self.config['maps'][0]['name']]['x']

        result = dict()
        for _solver_ in self.config['solvers']:
            result[_solver_['name']] = dict()
            for _map_ in self.config['maps']:
                result[_solver_['name']][_map_['name']] = {'x': list(), 'val': list(), 'ci': list()}
                for idx, _x_ in enumerate(x_list):
                    tmp_val1 = val1[_solver_['name']][_map_['name']]['val'][idx]
                    tmp_val2 = val2[_solver_['name']][_map_['name']]['val'][idx]

                    if use_op == 'add':
                        tmp_val = tmp_val1 + tmp_val2
                    elif use_op == 'sub':
                        tmp_val = tmp_val1 - tmp_val2
                    elif use_op == 'mil':
                        tmp_val = tmp_val1 * tmp_val2
                    elif use_op == 'div':
                        if tmp_val2 == 0:
                            tmp_val = np.inf
                        else:
                            tmp_val = float(tmp_val1) / float(tmp_val2)
                    elif use_op == 'mod':
                        if tmp_val2 == 0:
                            tmp_val = np.inf
                        else:
                            tmp_val = float(tmp_val1) % float(tmp_val2)
                        
                    result[_solver_['name']][_map_['name']]['x'].append(_x_)
                    result[_solver_['name']][_map_['name']]['val'].append(tmp_val)

        # Plot all the subplots on the figure
        fig, axs = plt.subplots(nrows=self.fig_axs[len(self.config['maps'])][0],
                                ncols=self.fig_axs[len(self.config['maps'])][1],
                                figsize=self.fig_size,
                                dpi=80, facecolor='w', edgecolor='k')

        for idx, _map_ in enumerate(self.config['maps']):
            frow, fcol = self.get_subfig_pos(idx)
            if len(self.config['maps']) == 1:
                self.subplot_fig(x_index, use_op, axs, idx, _map_, result)
            elif self.fig_axs[len(self.config['maps'])][0] == 1:
                self.subplot_fig(x_index, use_op, axs[fcol], idx, _map_, result)
            else:
                self.subplot_fig(x_index, use_op, axs[frow,fcol], idx, _map_, result)

        fig.tight_layout()

        if self.config['set_legend']:
            if use_op == 'div':
                plt.legend(loc="lower right", fontsize=self.text_size)
            else:
                plt.legend(loc="best", fontsize=self.text_size)

        fig_name = x_index + '_' + use_op + '_plot.png'
        plt.savefig(fig_name)
        plt.show()

    # def plot_hist_fig(self, x_index:str='num', y_index:List[str]=['num_ex_conf', 'num_in_conf']):
    #     # Get the result from the experiments
    #     results_list = list()
    #     for y_idx in y_index:
    #         result = self.get_val(x_index, y_idx)
    #         results_list.append(result)

    #     # Plot all the subplots on the figure
    #     fig, axs = plt.subplots(nrows=self.fig_axs[len(self.config['maps'])][0],
    #                             ncols=self.fig_axs[len(self.config['maps'])][1],
    #                             figsize=self.fig_size,
    #                             dpi=80, facecolor='w', edgecolor='k')

    #     for idx, _map_ in enumerate(self.config['maps']):
    #         frow, fcol = self.get_subfig_pos(idx)
    #         if len(self.config['maps']) == 1:
    #             self.subplot_fig(x_index, y_index, axs, idx, _map_, results_list)
    #         elif self.fig_axs[len(self.config['maps'])][0] == 1:
    #             self.subplot_fig(x_index, y_index, axs[fcol], idx, _map_, results_list)
    #         else:
    #             self.subplot_fig(x_index, y_index, axs[frow,fcol], idx, _map_, results_list)

    #     fig.tight_layout()
    #     if y_index == 'succ':
    #         plt.legend(loc="lower left", fontsize=self.text_size)
    #     elif y_index == 'runtime' or y_index == '#low-level generated' or \
    #           y_index == '#high-level generated':
    #         plt.legend(loc="upper left", fontsize=self.text_size)
    #     else:
    #         plt.legend(loc="best", fontsize=self.text_size)
    #     fig_name = x_index + '_' + y_index + '_plot.png'
    #     plt.savefig(fig_name)
    #     plt.show()

    def get_ins_from_samples(self, sol_dir:str, sol_names:List[str], 
                             mode:str='min', objective:str='runtime'):
        for _map_ in self.config['maps']:
            for _ag_num_ in _map_['num_of_agents']:
                for _scen_ in _map_['scens']:
                    util.create_csv_file(exp_path=self.config['exp_path'],
                                         map_name=_map_['name'],
                                         scen=_scen_,
                                         ag_num=_ag_num_,
                                         ins_num=self.config['ins_num'],
                                         sol_dir=sol_dir,
                                         sol_names=sol_names,
                                         mode=mode,
                                         objective=objective)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Take config.yaml as input!')
    parser.add_argument('--config', type=str, default='config.yaml')

    args = parser.parse_args()

    # Create data processor
    data_processor = DataProcessor(args.config)

    # Filtering from random (random) sampled solvers
    SOLVER_DIR = 'PBS_rand'
    solver_names = ['PBS1_0','PBS1_1','PBS1_2', 'PBS1_3','PBS1_4',
                    'PBS1_5','PBS1_6','PBS1_7','PBS1_8','PBS1_9']
    for _m_ in ['min', 'mid', 'max']:
        data_processor.get_ins_from_samples(sol_dir=SOLVER_DIR, sol_names=solver_names, mode=_m_)

    # data_processor.plot_fig(x_index='num', y_index='succ')
    # data_processor.plot_fig(x_index='num', y_index='runtime')
    # data_processor.plot_fig(x_index='num', y_index='max_ma_size')
    # data_processor.plot_fig(x_index='num', y_index='#low-level generated')
    # data_processor.plot_fig(x_index='num', y_index='#high-level generated')
    # data_processor.plot_fig(x_index='num', y_index='#pathfinding')

    # data_processor.plot_fig(x_index='ins', y_index='max_ma_size')
    # data_processor.plot_op(x_index='ins',y_index1='#pathfinding',
    #                        y_index2='#high-level generated',use_op='div')
    # data_processor.plot_fig(x_index='ins', y_index='solution cost')
    # data_processor.plot_fig(x_index='ins', y_index='num_0child')

    # data_processor.plot_fig(x_index='ins', y_index='num_total_conf')
    # data_processor.plot_fig(x_index='ins', y_index='num_in_conf')
    # data_processor.plot_fig(x_index='ins', y_index='num_ex_conf')
