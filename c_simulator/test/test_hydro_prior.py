#!/usr/bin/env python3
"""Compare ellipsoid quadrature with independently analytic limiting shapes."""
import importlib.util
from pathlib import Path
import unittest
import numpy as np

spec = importlib.util.spec_from_file_location('prior', Path(__file__).resolve().parents[1]/'scripts/generate_hydro_prior.py')
prior = importlib.util.module_from_spec(spec)
spec.loader.exec_module(prior)


class AddedMassPrior(unittest.TestCase):
    def test_sphere(self):
        volume = 4*np.pi/3*.2**3
        added = prior.ellipsoid_added_mass(1000, volume, [.2]*3)
        np.testing.assert_allclose(np.diag(added), [1000*volume/2]*3+[0.]*3, atol=2e-7)

    def test_prolate_imlay(self):
        a, b, rho = .5, .1, 1000
        volume = 4*np.pi/3*a*b*b
        e = np.sqrt(1-(b/a)**2)
        alpha = 2*(1-e*e)/e**3*(np.log((1+e)/(1-e))/2-e)
        beta = 1-alpha/2
        k1, k2 = alpha/(2-alpha), beta/(2-beta)
        kp = e**4*(beta-alpha)/((2-e*e)*(2*e*e-(2-e*e)*(beta-alpha)))
        mass = rho*volume
        inertia = mass*(a*a+b*b)/5
        expected = [mass*k1, mass*k2, mass*k2, 0, kp*inertia, kp*inertia]
        np.testing.assert_allclose(np.diag(prior.ellipsoid_added_mass(rho, volume, [a,b,b])), expected, atol=2e-7)


if __name__ == '__main__':
    unittest.main()
