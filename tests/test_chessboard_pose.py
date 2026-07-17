import numpy as np
import unittest

from scripts.chessboard_pose_publisher import create_object_points


class ChessboardPoseTest(unittest.TestCase):
    def test_create_object_points_uses_inner_corners_and_square_size(self):
        points = create_object_points(cols=8, rows=11, square_size=0.02)

        self.assertEqual(points.shape, (88, 3))
        np.testing.assert_allclose(points[0], [0.0, 0.0, 0.0])
        np.testing.assert_allclose(points[1], [0.02, 0.0, 0.0])
        np.testing.assert_allclose(points[8], [0.0, 0.02, 0.0])
        np.testing.assert_allclose(points[-1], [0.14, 0.20, 0.0])


if __name__ == "__main__":
    unittest.main()
