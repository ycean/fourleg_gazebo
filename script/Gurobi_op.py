#!/usr/bin/env python
__author__ = "Min Sung Ahn"
__email__ = "aminsung@gmail.com"
__copyright__ = "Copyright 2016 RoMeLa"
__date__ = "June 10, 2017"

__version__ = "0.0.1"
__status__ = "Prototype"
#modify for continuous fs plan and optimization


import numpy as np
import matplotlib.pyplot as plt
import matplotlib.collections as mc
import matplotlib.animation as ani
from math import pi, cos, sin, atan2, hypot
import sys
import gurobipy as gurobipy
import time

class QPBodyTrajectory(object):
    def __init__(self, dt=10):
        self.g = 9.81 # Gravity
        self.z_m = 0.1952 # 0.4 # [m] COG height
        # TODO: Adjustable # of spline count
        self.nsp = 6 # Order of spline
        self.nq = 3 # No. of quintic splines
        self.T = 1 # Duration of spline
        self.t = np.linspace(0.0, self.T, dt) # Discretization # Originally 100...
        self.com_x = []
        self.com_y = []
        self.mkr = []

        self.T2 = self.T*self.T
        self.T3 = self.T2*self.T
        self.T4 = self.T3*self.T
        self.T5 = self.T4*self.T
        self.T6 = self.T5*self.T
        self.T7 = self.T6*self.T

        # Cost Pre-processing
        #self.G = np.array([[(400.0/7.0)*self.T7, 40.0*self.T6, (120.0/5.0)*self.T5, 10.0*self.T4], \
        #                   [40.0*self.T6, (144.0/5.0)*self.T5, 18.0*self.T4, 8.0*self.T3], \
        #                   [(120.0/5.0)*self.T5, 18.0*self.T4, 12.0*self.T3, 6.0*self.T2], \
        #                   [10.0*self.T4, 8.0*self.T3, 6.0*self.T2, 4.0*self.T]])

        #self.G0x = self.G[0, :].tolist()
        #self.G1x = self.G[1, :].tolist()
        #self.G2x = self.G[2, :].tolist()
        #self.G3x = self.G[3, :].tolist()

        # Equality Pre-processing
        self.cf_pos = [self.T5, self.T4, self.T3, self.T2, self.T, 1.0]
        self.cf_vel = [5.0*self.T4, 4.0*self.T3, 3.0*self.T2, 2.0*self.T, 1.0]
        self.cf_acc = [20.0*self.T3, 12.0*self.T2, 6.0*self.T, 2.0]

        # Inequality Pre-processing
        self.zmp_0_list = [
            0.0 - (20.0 * self.z_m * 0.0) / self.g, 0.0 - (12.0 * self.z_m * 0.0) / self.g, \
            0.0 - (6.0 * self.z_m * 0.0) / self.g, 0.0 - (2.0 * self.z_m) / self.g, \
            0.0, 1.0
        ]
        self.zmp_T_list = [
            self.T5 - (20.0 * self.z_m * self.T3) / self.g, self.T4 - (12.0 * self.z_m * self.T2) / self.g, \
            self.T3 - (6.0 * self.z_m * self.T) / self.g, self.T2 - (2.0 * self.z_m) / self.g, \
            self.T, 1.0
        ]

    def _update_footsteps(self, fs, p0=None, pf=None):
        # After creating the QPBodyTrajectory object, update the footsteps for the QP to solve
        # p0: Initial center of the support polygon
        # pf: Final center of the support polygon

        self.fs = fs

        fs_start = 1
        fs_next = 4

        if p0 == None:
            self.p0 = np.sum(self.fs[:, fs_start:fs_start + 3] / 3.0, 1, keepdims=True)

        if pf == None:
            self.pf = np.sum(self.fs[:, fs_next:fs_next + 3] / 3.0, 1, keepdims=True)
        # print("Initial Footstep:\n{}\nFinal Footstep:\n{}".format(self.p0, self.pf))


    def _gen_vec(self):
        # Convert to arrays
        self.vec_cf_pos = np.asarray(self.cf_pos).reshape(6, 1)
        self.vec_cf_vel = np.asarray(self.cf_vel).reshape(5, 1)
        self.vec_cf_acc = np.asarray(self.cf_acc).reshape(4, 1)

        # Convert coefficients to arrays
        self.vec_x1_q1 = np.asarray([self.x1_q1[k].X for k in range(6)]).reshape(6, 1)
        self.vec_x1_q2 = np.asarray([self.x1_q2[k].X for k in range(6)]).reshape(6, 1)
        self.vec_x1_q3 = np.asarray([self.x1_q3[k].X for k in range(6)]).reshape(6, 1)
        self.vec_y1_q1 = np.asarray([self.y1_q1[k].X for k in range(6)]).reshape(6, 1)
        self.vec_y1_q2 = np.asarray([self.y1_q2[k].X for k in range(6)]).reshape(6, 1)
        self.vec_y1_q3 = np.asarray([self.y1_q3[k].X for k in range(6)]).reshape(6, 1)
        self.vec_x2_q1 = np.asarray([self.x2_q1[k].X for k in range(6)]).reshape(6, 1)
        self.vec_x2_q2 = np.asarray([self.x2_q2[k].X for k in range(6)]).reshape(6, 1)
        self.vec_x2_q3 = np.asarray([self.x2_q3[k].X for k in range(6)]).reshape(6, 1)
        self.vec_y2_q1 = np.asarray([self.y2_q1[k].X for k in range(6)]).reshape(6, 1)
        self.vec_y2_q2 = np.asarray([self.y2_q2[k].X for k in range(6)]).reshape(6, 1)
        self.vec_y2_q3 = np.asarray([self.y2_q3[k].X for k in range(6)]).reshape(6, 1)


    def run_qp_model(self, fs, origin_start=False): #start_loop
        self.fs = fs
        self._update_footsteps(self.fs) # Updates self.fs
        print(str(self.fs))

        p1, q1, r1 = self._get_support_polygon(3, self.fs[:,1:4])
        p2, q2, r2 = self._get_support_polygon(3, self.fs[:,4:7])

        # Cost Pre-processing
        self.G = np.array([[(400.0/7.0)*self.T7, 40.0*self.T6, (120.0/5.0)*self.T5, 10.0*self.T4], \
                           [40.0*self.T6, (144.0/5.0)*self.T5, 18.0*self.T4, 8.0*self.T3], \
                           [(120.0/5.0)*self.T5, 18.0*self.T4, 12.0*self.T3, 6.0*self.T2], \
                           [10.0*self.T4, 8.0*self.T3, 6.0*self.T2, 4.0*self.T]])

        self.G0x = self.G[0, :].tolist()
        self.G1x = self.G[1, :].tolist()
        self.G2x = self.G[2, :].tolist()
        self.G3x = self.G[3, :].tolist()

        # === Create Model
        self.m = gurobipy.Model("cog_and_zmp_qp")
        self.m.setParam('MIPGap', 0)
        self.m.setParam('BarHomogeneous', 1)
        self.m.setParam('OutputFlag', 1)

        # === Create Variables
        self.x1_q1 = self.gen_vvar(self.m, 'x1_q1', 6)
        self.x1_q2 = self.gen_vvar(self.m, 'x1_q2', 6)
        self.x1_q3 = self.gen_vvar(self.m, 'x1_q3', 6)
        self.x2_q1 = self.gen_vvar(self.m, 'x2_q1', 6)
        self.x2_q2 = self.gen_vvar(self.m, 'x2_q2', 6)
        self.x2_q3 = self.gen_vvar(self.m, 'x2_q3', 6)
        self.y1_q1 = self.gen_vvar(self.m, 'y1_q1', 6)
        self.y1_q2 = self.gen_vvar(self.m, 'y1_q2', 6)
        self.y1_q3 = self.gen_vvar(self.m, 'y1_q3', 6)
        self.y2_q1 = self.gen_vvar(self.m, 'y2_q1', 6)
        self.y2_q2 = self.gen_vvar(self.m, 'y2_q2', 6)
        self.y2_q3 = self.gen_vvar(self.m, 'y2_q3', 6)

        self.m.update()

        # === Create cost function
        self.cost_expr = gurobipy.QuadExpr()
        self.add_single_spline_acc_sq_cost(self.x1_q1, self.cost_expr)
        self.add_single_spline_acc_sq_cost(self.x1_q2, self.cost_expr)
        self.add_single_spline_acc_sq_cost(self.x1_q3, self.cost_expr)
        self.add_single_spline_acc_sq_cost(self.x2_q1, self.cost_expr)
        self.add_single_spline_acc_sq_cost(self.x2_q2, self.cost_expr)
        self.add_single_spline_acc_sq_cost(self.x2_q3, self.cost_expr)
        self.add_single_spline_acc_sq_cost(self.y1_q1, self.cost_expr)
        self.add_single_spline_acc_sq_cost(self.y1_q2, self.cost_expr)
        self.add_single_spline_acc_sq_cost(self.y1_q3, self.cost_expr)
        self.add_single_spline_acc_sq_cost(self.y2_q1, self.cost_expr)
        self.add_single_spline_acc_sq_cost(self.y2_q2, self.cost_expr)
        self.add_single_spline_acc_sq_cost(self.y2_q3, self.cost_expr)

        # === Set objective function
        self.m.setObjective(self.cost_expr, gurobipy.GRB.MINIMIZE)
        self.m.update()

        # === Add equality constraints
        # ===== Initial Condition
        if origin_start:
            self.m.addConstr(gurobipy.LinExpr([0.0, 0.0, 0.0, 0.0, 0.0, 1.0], self.x1_q1) == 0.0)
            self.m.addConstr(gurobipy.LinExpr([0.0, 0.0, 0.0, 0.0, 0.0, 1.0], self.y1_q1) == 0.0)
        else:
            self.m.addConstr(gurobipy.LinExpr([0.0, 0.0, 0.0, 0.0, 0.0, 1.0], self.x1_q1) == self.p0[0, 0])
            self.m.addConstr(gurobipy.LinExpr([0.0, 0.0, 0.0, 0.0, 0.0, 1.0], self.y1_q1) == self.p0[1, 0])

        # ===== Final Condition
        self.m.addConstr(gurobipy.LinExpr(self.cf_pos, self.x2_q3) == self.pf[0, 0])
        self.m.addConstr(gurobipy.LinExpr(self.cf_pos, self.y2_q3) == self.pf[1, 0])

        # ===== X 1
        self.m.addConstr(gurobipy.LinExpr(self.cf_pos, self.x1_q1) == self.x1_q2[5], "con_x1_q1_pos")
        self.m.addConstr(gurobipy.LinExpr(self.cf_pos, self.x1_q2) == self.x1_q3[5], "con_x1_q2_pos")
        self.m.addConstr(gurobipy.LinExpr(self.cf_vel, self.x1_q1[0:5]) == self.x1_q2[4], "con_x1_q1_vel")
        self.m.addConstr(gurobipy.LinExpr(self.cf_vel, self.x1_q2[0:5]) == self.x1_q3[4], "con_x1_q2_vel")
        self.m.addConstr(gurobipy.LinExpr(self.cf_acc, self.x1_q1[0:4]) == 2.0 * self.x1_q2[3], "con_x1_q1_acc")
        self.m.addConstr(gurobipy.LinExpr(self.cf_acc, self.x1_q2[0:4]) == 2.0 * self.x1_q3[3], "con_x1_q2_acc")

        # ===== Y 1
        self.m.addConstr(gurobipy.LinExpr(self.cf_pos, self.y1_q1) == self.y1_q2[5], "con_y1_q1_pos")
        self.m.addConstr(gurobipy.LinExpr(self.cf_pos, self.y1_q2) == self.y1_q3[5], "con_y1_q2_pos")
        self.m.addConstr(gurobipy.LinExpr(self.cf_vel, self.y1_q1[0:5]) == self.y1_q2[4], "con_y1_q1_vel")
        self.m.addConstr(gurobipy.LinExpr(self.cf_vel, self.y1_q2[0:5]) == self.y1_q3[4], "con_y1_q2_vel")
        self.m.addConstr(gurobipy.LinExpr(self.cf_acc, self.y1_q1[0:4]) == 2.0 * self.y1_q2[3], "con_y1_q1_acc")
        self.m.addConstr(gurobipy.LinExpr(self.cf_acc, self.y1_q2[0:4]) == 2.0 * self.y1_q3[3], "con_y1_q2_acc")

        # ===== X 2
        self.m.addConstr(gurobipy.LinExpr(self.cf_pos, self.x2_q1) == self.x2_q2[5], "con_x2_q1_pos")
        self.m.addConstr(gurobipy.LinExpr(self.cf_pos, self.x2_q2) == self.x2_q3[5], "con_x2_q2_pos")
        self.m.addConstr(gurobipy.LinExpr(self.cf_vel, self.x2_q1[0:5]) == self.x2_q2[4], "con_x2_q1_vel")
        self.m.addConstr(gurobipy.LinExpr(self.cf_vel, self.x2_q2[0:5]) == self.x2_q3[4], "con_x2_q2_vel")
        self.m.addConstr(gurobipy.LinExpr(self.cf_acc, self.x2_q1[0:4]) == 2.0 * self.x2_q2[3], "con_x2_q1_acc")
        self.m.addConstr(gurobipy.LinExpr(self.cf_acc, self.x2_q2[0:4]) == 2.0 * self.x2_q3[3], "con_x2_q2_acc")

        # ===== Y 2
        self.m.addConstr(gurobipy.LinExpr(self.cf_pos, self.y2_q1) == self.y2_q2[5], "con_y2_q1_pos")
        self.m.addConstr(gurobipy.LinExpr(self.cf_pos, self.y2_q2) == self.y2_q3[5], "con_y2_q2_pos")
        self.m.addConstr(gurobipy.LinExpr(self.cf_vel, self.y2_q1[0:5]) == self.y2_q2[4], "con_y2_q1_vel")
        self.m.addConstr(gurobipy.LinExpr(self.cf_vel, self.y2_q2[0:5]) == self.y2_q3[4], "con_y2_q2_vel")
        self.m.addConstr(gurobipy.LinExpr(self.cf_acc, self.y2_q1[0:4]) == 2.0 * self.y2_q2[3], "con_y2_q1_acc")
        self.m.addConstr(gurobipy.LinExpr(self.cf_acc, self.y2_q2[0:4]) == 2.0 * self.y2_q3[3], "con_y2_q2_acc")

        # ===== Equality between triangles
        self.m.addConstr(gurobipy.LinExpr(self.cf_pos, self.x1_q3) == self.x2_q1[5], "con_x1_tri_pos")
        self.m.addConstr(gurobipy.LinExpr(self.cf_pos, self.y1_q3) == self.y2_q1[5], "con_y1_tri_pos")
        self.m.addConstr(gurobipy.LinExpr(self.cf_vel, self.x1_q3[0:5]) == self.x2_q1[4], "con_x1_tri_vel")
        self.m.addConstr(gurobipy.LinExpr(self.cf_vel, self.y1_q3[0:5]) == self.y2_q1[4], "con_y1_tri_vel")

        for idx in range(3):
            self.m.addConstr(
                gurobipy.LinExpr(
                    [p1[idx,0] * jdx for jdx in self.zmp_0_list] + [q1[idx,0] * jdx for jdx in self.zmp_0_list],
                    self.x1_q1 + self.y1_q1) <= -r1[idx, 0], "zmp_x1_"+str(idx))
            self.m.addConstr(
                gurobipy.LinExpr(
                    [p1[idx, 0] * jdx for jdx in self.zmp_T_list] + [q1[idx,0] * jdx for jdx in self.zmp_T_list],
                    self.x1_q3 + self.y1_q3) <= -r1[idx, 0], "zmp_y1_"+str(idx))
            self.m.addConstr(
                gurobipy.LinExpr(
                    [p2[idx, 0] * jdx for jdx in self.zmp_0_list] + [q2[idx,0] * jdx for jdx in self.zmp_0_list],
                    self.x2_q1 + self.y2_q1) <= -r2[idx, 0], "zmp_x2_"+str(idx))
            self.m.addConstr(
                gurobipy.LinExpr(
                    [p2[idx, 0] * jdx for jdx in self.zmp_T_list] + [q2[idx,0] * jdx for jdx in self.zmp_T_list],
                    self.x2_q3 + self.y2_q3) <= -r2[idx, 0], "zmp_y2_"+str(idx))

        self.m.update()

        self.m.optimize()

        print("Optimization Done")

        XCOM , YCOM = self.get_com_trajectory()

        return XCOM , YCOM

    def gen_vvar(self, model, name, size):
        vec = []
        for idx in range(size):
            nm = name + "_" + str(idx)
            vec.append(model.addVar(lb=-gurobipy.GRB.INFINITY, ub=gurobipy.GRB.INFINITY, name=nm))
        return vec

    def add_single_spline_acc_sq_cost(self, vec, expr=None):
        expr.addTerms(self.G0x, [vec[0] for _ in range(4)], vec[0:4])
        expr.addTerms(self.G1x, [vec[1] for _ in range(4)], vec[0:4])
        expr.addTerms(self.G2x, [vec[2] for _ in range(4)], vec[0:4])
        expr.addTerms(self.G3x, [vec[3] for _ in range(4)], vec[0:4])

    def get_trajectory(self):
        fs_start = 1
        fs_next = 4
        p, q, r = self._get_support_polygon(self.fs[:,fs_start:fs_start+2])
        p2, q2, r2 = self._get_support_polygon(self.fs[:, fs_next:fs_next + 2])


    def _get_support_polygon(self, n_pts, pts):
        """
        Given a number of points in the polygon, find p, q, r, where the line is defined as px + qy + r = 0.

        :param n_pts: Number of points to make a support polygon
        :param pts: Points to make the support polygon out of
        :return: p, q, and r values that define the lines of the polygon
        """

        p = np.zeros((n_pts,1))
        q = np.ones((n_pts,1))
        r = np.zeros((n_pts,1))
        pts_m = np.vstack(((pts[:,0,None] + pts[:,1,None] + pts[:,2,None])/3, np.array([1.0])))

        for idx in range(n_pts):
            if idx == n_pts-1:
                p[idx,None], r[idx,None] = self._get_line_equation(pts[:,idx], pts[:,0])
            else:
                p[idx,None], r[idx,None] = self._get_line_equation(pts[:,idx], pts[:,idx+1])

            p[idx,None] = -p[idx,None]
            r[idx,None] = -r[idx,None]

            if ((np.array([p[idx], q[idx], r[idx]]).T.dot(pts_m)[0] >= 0)[0]):
                p[idx,None] = -p[idx,None]
                q[idx,None] = -q[idx,None]
                r[idx,None] = -r[idx,None]

        return p, q, r

    def _get_line_equation(self, pt1, pt2):
        m = (pt2[1]-pt1[1]) / (pt2[0]-pt1[0])
        b = pt1[1] - m*pt1[0]
        return m, b

    def get_com_seg_trajectory(self, x, y, z_m, t):
        x_pt = x[0].X * (t ** 5) + x[1].X * (t ** 4) + x[2].X * (t ** 3) + x[3].X * (t ** 2) + x[4].X * (t) + x[5].X
        y_pt = y[0].X * (t ** 5) + y[1].X * (t ** 4) + y[2].X * (t ** 3) + y[3].X * (t ** 2) + y[4].X * (t) + y[5].X

        return x_pt, y_pt

    def get_com_trajectory(self):
        x11, y11 = self.get_com_seg_trajectory(self.x1_q1, self.y1_q1, self.z_m, self.t)
        x12, y12 = self.get_com_seg_trajectory(self.x1_q2, self.y1_q2, self.z_m, self.t)
        x13, y13 = self.get_com_seg_trajectory(self.x1_q3, self.y1_q3, self.z_m, self.t)
        x21, y21 = self.get_com_seg_trajectory(self.x2_q1, self.y2_q1, self.z_m, self.t)
        x22, y22 = self.get_com_seg_trajectory(self.x2_q2, self.y2_q2, self.z_m, self.t)
        x23, y23 = self.get_com_seg_trajectory(self.x2_q3, self.y2_q3, self.z_m, self.t)
        x1_2 = np.append(x11, x12,0)
        x1_3 = np.append(x1_2, x13,0)
        x12_1 = np.append(x1_3, x21,0)
        x12_2 = np.append(x12_1, x22,0)
        xcom = np.append(x12_2, x23,0)
        y1_2 = np.append(y11, y12,0)
        y1_3 = np.append(y1_2, y13,0)
        y12_1 = np.append(y1_3, y21,0)
        y12_2 = np.append(y12_1, y22,0)
        ycom = np.append(y12_2, y23,0)
        return xcom, ycom

    def get_zmp_trajectory(self, x, y, z_m, t):
        xcom, ycom = self.get_com_trajectory(x, y, z_m, t)
        xddcom = 20.0 * x[0].X * (t ** 3) + 12.0 * x[1].X * (t ** 2) + 6.0 * x[2].X * (t) + 2.0 * x[3].X
        yddcom = 20.0 * y[0].X * (t ** 3) + 12.0 * y[1].X * (t ** 2) + 6.0 * y[2].X * (t) + 2.0 * y[3].X
        xzmp = xcom - (z_m * xddcom) / 9.81
        yzmp = ycom - (z_m * yddcom) / 9.81
        return xzmp, yzmp

    def plot_trajectory(self, x, y, mkr='b-'):
        plt.cla()
        plt.plot(x, y)
        min_x = min(x) - 0.5
        max_x = max(x) + 0.5
        min_y = min(y) - 0.5
        max_y = max(y) + 0.5
        plt.axis([min_x, max_x, min_y, max_y])


# === Test code
if __name__ == '__main__':

    demo = QPBodyTrajectory()
    demo.run_qp_model()
