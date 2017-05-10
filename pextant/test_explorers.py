import unittest
import numpy as np
from explorers import Astronaut

class TestEnergyCostFunctions(unittest.TestCase):
    def _generate_slopes(self):
        slopes = np.radians(np.linspace(-25,25,100))
        return slopes

    def test_slope_energy_cost(self):
        slopes_rad = self._generate_slopes()
        slopes_deg = np.degrees(slopes_rad)
        explorer = Astronaut(80)
        dl = 0.5 # path length
        g = 9.81 # gravity

        total_cost_quick, _ = explorer.total_energy_cost(dl, slopes_rad, g)

        time_cost = explorer.time(dl, slopes_deg)
        energy_rate = explorer.energyRate(dl, slopes_deg, g)
        total_cost_long = time_cost * energy_rate

        self.assertAlmostEqual(total_cost_quick[0], total_cost_long[0])

if __name__ == '__main__':
    unittest.main()