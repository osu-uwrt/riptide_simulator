#!/usr/bin/env python3
"""Generate explicit UNVALIDATED Talos starting values. Never writes hardware config.

Potential-flow equivalent ellipsoid; translation uses Lamb's ellipsoidal factors.
Rotation reduces to the prolate Imlay formula in MSS imlay61.m. The body envelope
is scaled to the displaced volume before evaluating added inertia. Open-frame
entrainment, viscous effects and thruster interactions require real identification.
"""
import argparse
from pathlib import Path
import numpy as np
import yaml


def ellipsoid_added_mass(density, volume, radii):
    radii = np.array(radii, dtype=float, copy=True)
    radii *= (volume/(4*np.pi/3*np.prod(radii)))**(1/3)
    # Gauss-Legendre on [0,1], s=L²*t/(1-t) maps to the infinite integral.
    nodes, weights = np.polynomial.legendre.leggauss(256)
    t, weights = (nodes+1)/2, weights/2
    scale = max(radii)**2
    s = scale*t/(1-t)
    jac = scale/(1-t)**2
    squared = radii**2
    delta = np.sqrt(np.prod(s[:, None]+squared[None, :], axis=1))
    alpha = np.prod(radii)*np.sum(
        (weights*jac/delta)[:, None]/(s[:, None]+squared[None, :]), axis=0)
    mass = density*volume
    diagonal = list(mass*alpha/(2-alpha))
    for j, k in ((1, 2), (2, 0), (0, 1)):
        difference = squared[j]-squared[k]
        if abs(difference) < 1e-12:
            diagonal.append(0.)
        else:
            denominator = 2*difference-(squared[j]+squared[k])*(alpha[k]-alpha[j])
            diagonal.append(mass/5*difference**2*(alpha[k]-alpha[j])/denominator)
    return np.diag(diagonal)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--vehicle-config', required=True)
    parser.add_argument('--legacy-simulator-config', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    vehicle = yaml.safe_load(Path(args.vehicle_config).read_text())
    legacy = yaml.safe_load(Path(args.legacy_simulator_config).read_text())['vehicle_properties']
    rho = 998.2
    volume = (vehicle['mass'] + 3/9.80665)/rho  # 3 N positive buoyancy, an unverified starting assumption
    radii = [.175, .415, .275]  # existing Talos collision envelope, NOT sealed-volume measurements
    added = ellipsoid_added_mass(rho, volume, radii)
    # Preserve the rough low-speed response of the existing simulator curves,
    # using a dissipative linear + quadratic approximation independent of control gains.
    linear, quadratic = [], []
    coefficients = np.asarray(legacy['damping']).reshape(6, 4)
    for i, (_, b, c, d) in enumerate(coefficients):
        v = np.linspace(0, .6 if i < 3 else 1., 200)
        force = np.maximum(0., b*v+c*np.expm1(v/d))
        X = np.column_stack((v, v*v))
        candidates = [np.maximum(np.linalg.lstsq(X, force, rcond=None)[0], 0),
                      np.array([max(0., v@force/(v@v)), 0.]),
                      np.array([0., max(0., (v*v)@force/((v*v)@(v*v)))])]
        fit = min(candidates, key=lambda x: np.linalg.norm(X@x-force))
        linear.append(float(fit[0]));quadratic.append(float(fit[1]))
    report = dict(schema_version=1, robot='talos', parameter_status='unvalidated_prior',
                  provenance='CAD rigid inertia; equivalent-ellipsoid added mass; approximated legacy drag. NO pool identification.',
                  rigid_body_inertia3x3=legacy['rigid_body_inertia3x3'],
                  added_mass6x6=added.tolist(), linear_damping6x6=np.diag(linear).tolist(),
                  quadratic_damping=quadratic, damping_center_relative=[0.,0.,0.],
                  water_density=rho, displaced_volume=volume, cob_relative=legacy['cob_relative'],
                  buoyancy_radii=radii, water_level=0.,
                  current_velocity=[0.,0.,0.], current_oscillation_amplitude=[0.,0.,0.],
                  current_oscillation_frequency=.1,
                  thruster_dynamics=dict(legacy['thruster_dynamics'],command_timeout=.5,
                                         forward_max_force=legacy['thruster_max_force'],
                                         reverse_max_force=28.,propeller_radius=.05),
                  thruster_efficiencies=[1.]*len(vehicle['thrusters']))
    Path(args.output).write_text('# UNVALIDATED starting model. See docs/PHYSICS.md before gain transfer.\n'+yaml.safe_dump(report, sort_keys=False))
    print('Added mass diagonal:', np.diag(added))
    print('Linear damping:',linear,'Quadratic damping:',quadratic)


if __name__ == '__main__':
    main()
