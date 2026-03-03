# -*- coding: utf-8 -*-
"""LPV H-infinity controller with rho-scheduled performance weighting.

Author: Suwon Lee, Kookmin University
Created: 2026-03-01
Description: Polytopic LPV H-infinity controller that interpolates between
    6 vertex controllers based on rho = |kappa| * v.

Sign convention note:
    The H-infinity synthesis in MATLAB uses e_psi = psi - psi_des.
    This controller internally handles the sign inversion so that
    users always provide e_psi = psi_des - psi (standard convention).

Uses Tustin (bilinear) discretization for unconditional stability.
"""

import json
from importlib import resources
import numpy as np
from scipy.signal import cont2discrete
from .base import ControllerBase


class LPVHinfController(ControllerBase):
    """Polytopic LPV H-infinity controller.

    Interpolates between vertex controllers based on the scheduling
    parameter rho = |kappa| * v.

    Parameters
    ----------
    vertices : list of float
        Scheduling parameter values at each vertex.
    controllers : list of dict
        Each dict has keys 'A', 'B', 'C', 'D' (continuous-time state-space).
    dt : float
        Sample time for Tustin discretization [s]. Default 0.01.
    """

    def __init__(self, vertices, controllers, dt=0.01):
        self.vertices = np.array(vertices, dtype=float)
        self.n_vertices = len(vertices)
        self.dt = dt

        # Tustin-discretize each vertex controller
        self.controllers = []
        for ctrl in controllers:
            Ad, Bd, Cd, Dd, _ = cont2discrete(
                (ctrl['A'], ctrl['B'], ctrl['C'], ctrl['D']),
                dt, method='bilinear')
            self.controllers.append({'Ad': Ad, 'Bd': Bd, 'Cd': Cd, 'Dd': Dd})

        # Internal state for each vertex controller
        self._states = []
        for ctrl in self.controllers:
            nx = ctrl['Ad'].shape[0]
            self._states.append(np.zeros(nx))

        self.version = None
        self.metadata = {}

    def reset(self):
        """Reset all internal controller states to zero."""
        for i, ctrl in enumerate(self.controllers):
            nx = ctrl['Ad'].shape[0]
            self._states[i] = np.zeros(nx)

    def compute(self, e_psi, delta_meas=0.0, rho=0.0, dt=0.01, **kwargs):
        """Compute steering command via polytopic interpolation.

        Sign convention: the user provides e_psi = psi_des - psi.
        Internally, the sign is inverted for the MATLAB-convention controller.

        Parameters
        ----------
        e_psi : float
            Heading error [rad]. Convention: psi_des - psi.
        delta_meas : float
            Measured steering angle [rad].
        rho : float
            Scheduling parameter |kappa| * v.
        dt : float
            Time step [s].

        Returns
        -------
        delta_cmd : float
            Steering command [rad].
        """
        # Invert sign for MATLAB convention: e_psi_matlab = psi - psi_des = -e_psi
        y = np.array([-e_psi, delta_meas])

        rho = np.clip(rho, self.vertices[0], self.vertices[-1])

        # Find bracketing vertices
        idx = np.searchsorted(self.vertices, rho, side='right') - 1
        idx = np.clip(idx, 0, self.n_vertices - 2)

        rho_lo = self.vertices[idx]
        rho_hi = self.vertices[idx + 1]

        if rho_hi - rho_lo > 1e-10:
            alpha = (rho - rho_lo) / (rho_hi - rho_lo)
        else:
            alpha = 0.0

        # Compute output from bracketing vertex controllers
        u_list = []
        for j in [idx, idx + 1]:
            ctrl = self.controllers[j]
            x = self._states[j]

            u_j = ctrl['Cd'] @ x + ctrl['Dd'] @ y
            self._states[j] = ctrl['Ad'] @ x + ctrl['Bd'] @ y
            u_list.append(u_j.flatten())

        # Interpolate
        u = (1.0 - alpha) * u_list[0] + alpha * u_list[1]

        if u.size == 1:
            return float(u[0])
        return u

    @classmethod
    def from_json(cls, json_path, dt=0.01):
        """Load from JSON file exported by MATLAB.

        Parameters
        ----------
        json_path : str
            Path to JSON file with keys 'vertices' and 'controllers'.
        dt : float
            Sample time for Tustin discretization [s].

        Returns
        -------
        controller : LPVHinfController
        """
        with open(json_path, 'r') as f:
            data = json.load(f)

        vertices = data['vertices']
        controllers = []
        for c in data['controllers']:
            ctrl = {
                'A': np.atleast_2d(np.array(c['A'], dtype=float)),
                'B': np.atleast_2d(np.array(c['B'], dtype=float)),
                'C': np.atleast_2d(np.array(c['C'], dtype=float)),
                'D': np.atleast_2d(np.array(c['D'], dtype=float)),
            }
            controllers.append(ctrl)

        obj = cls(vertices, controllers, dt=dt)
        obj.version = data.get('version', None)
        obj.metadata = {k: v for k, v in data.items()
                        if k not in ('vertices', 'controllers')}
        return obj

    @classmethod
    def from_dict(cls, data, dt=0.01):
        """Load from already-parsed controller dict."""
        vertices = data['vertices']
        controllers = []
        for c in data['controllers']:
            ctrl = {
                'A': np.atleast_2d(np.array(c['A'], dtype=float)),
                'B': np.atleast_2d(np.array(c['B'], dtype=float)),
                'C': np.atleast_2d(np.array(c['C'], dtype=float)),
                'D': np.atleast_2d(np.array(c['D'], dtype=float)),
            }
            controllers.append(ctrl)

        obj = cls(vertices, controllers, dt=dt)
        obj.version = data.get('version', None)
        obj.metadata = {k: v for k, v in data.items()
                        if k not in ('vertices', 'controllers')}
        return obj

    @classmethod
    def default(cls, dt=0.01):
        """Load the bundled v3 controller.

        Returns
        -------
        controller : LPVHinfController
            Pre-computed v3 controller with 6 vertices.
        """
        data = json.loads(
            resources.read_text(
                "qcar2_control.scalecar_vfg.data.controllers",
                "lpv_hinf_v3.json",
                encoding="utf-8",
            )
        )
        return cls.from_dict(data, dt=dt)
