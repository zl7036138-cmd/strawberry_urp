import math
import pathlib
import sys
import unittest
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from strawberry_manipulation.core import Pose
from strawberry_manipulation.carried_geometry import local_fruit_center, rotate, transit_hand_height


class CarriedGeometryTests(unittest.TestCase):
    def test_visual_center_transforms_into_hand_without_truth(self):
        hand = Pose(.35, .15, .648, qx=1, qw=0)
        local = local_fruit_center(hand, (.35, .15, .552))
        self.assertAlmostEqual(local[2], .096)
        self.assertAlmostEqual(rotate(local, hand)[2], -.096)

    def test_seed45504_low_corridor_is_raised_over_all_fruit(self):
        orientation = Pose(0, 0, 0, qx=1, qw=0)
        height = transit_hand_height(.6664, orientation, (0,0,.0964),
                                     [(0.348,.153,.552), (.318,-.141,.545)], .026, .015)
        self.assertAlmostEqual(height, .7304)
        # The old route left only 25 mm vertical centre separation; now the
        # carried envelope clears both radii and both uncertainty paddings.
        self.assertGreaterEqual(height-.0964-.545, 2*(.026+.015))

    def test_height_never_reduces_nominal_clearance(self):
        height = transit_hand_height(.9, Pose(0,0,0), (0,0,.096), [(0,0,.5)], .026, .015)
        self.assertEqual(height, .9)

    def test_rotation_and_inverse_round_trip(self):
        pose = Pose(0,0,0,qz=math.sin(.4),qw=math.cos(.4))
        point = (.1,.2,.3)
        back = rotate(rotate(point,pose), pose,inverse=True)
        for a,b in zip(point,back): self.assertAlmostEqual(a,b)

    def test_invalid_geometry_fails_closed(self):
        for centers in ([], [(0,0,float('nan'))], [(0,0)]):
            with self.assertRaises(ValueError):
                transit_hand_height(.7, Pose(0,0,0), (0,0,.1), centers, .026,.015)
        with self.assertRaises(ValueError): rotate((0,0,.1), Pose(0,0,0,qw=0))


if __name__ == '__main__': unittest.main()
