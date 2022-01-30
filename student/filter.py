# ---------------------------------------------------------------------
# Project "Track 3D-Objects Over Time"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Purpose of this file : Kalman filter class
#
# You should have received a copy of the Udacity license together with this program.
#
# https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
# ----------------------------------------------------------------------
#

# imports
import numpy as np

# add project directory to python path to enable relative imports
import os
import sys

PACKAGE_PARENT = ".."
SCRIPT_DIR = os.path.dirname(
    os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__)))
)
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))
import misc.params as params


class Filter:
    """Kalman filter class"""

    def __init__(self):
        pass

    def F(self):

        dt = params.dt
        return np.matrix(
            [
                [1, 0, 0, dt, 0, 0],
                [0, 1, 0, 0, dt, 0],
                [0, 0, 1, 0, 0, dt],
                [0, 0, 0, 1, 0, 0],
                [0, 0, 0, 0, 1, 0],
                [0, 0, 0, 0, 0, 1],
            ]
        )

    def Q(self):
        dt = params.dt
        q = params.q

        q3 = q * (dt ** 3) / 3
        q2 = q * (dt ** 2) / 2
        q1 = q * dt

        return np.matrix(
            [
                [q3, 0, 0, q2, 0, 0],
                [0, q3, 0, 0, q2, 0],
                [0, 0, q3, 0, 0, q2],
                [q2, 0, 0, q1, 0, 0],
                [0, q2, 0, 0, q1, 0],
                [0, 0, q2, 0, 0, q1],
            ]
        )

    def predict(self, track):
        x = track.x
        P = track.P
        F = self.F()
        Q = self.Q()

        x = F * x
        P = F * P * F.T + Q

        track.set_x(x)
        track.set_P(P)

    def update(self, track, meas):
        x = track.x
        P = track.P
        H = meas.sensor.get_H(x)
        gamma = self.gamma(track, meas)
        S = self.S(track, meas, H)
        I = np.eye(params.dim_state)

        K = P * H.T * np.linalg.inv(S)
        x = x + K * gamma
        P = (I - K * H) * P

        track.set_x(x)
        track.set_P(P)

        track.update_attributes(meas)

    def gamma(self, track, meas):
        z = meas.z
        hx = meas.sensor.get_hx(track.x)
        return z - hx

    def S(self, track, meas, H):
        R = meas.R
        P = track.P
        return H * P * H.T + R
