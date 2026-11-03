"""Offline triangle silhouette checks; no ROS, camera, or robot is contacted."""
from pathlib import Path
import struct
import sys
import tempfile
import unittest

import numpy as np

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
import cam_proj_validation as projection

try:
    import cv2
except ImportError:
    cv2 = None


def rectangle(x0, y0, x1, y1):
    return np.array([[[x0, y0], [x1, y0], [x1, y1]],
                     [[x0, y0], [x1, y1], [x0, y1]]], dtype=float)


class TriangleGeometryTests(unittest.TestCase):
    def test_stl_keeps_real_faces_and_shared_vertices(self):
        vertices = np.array([[[0, 0, 1], [1, 0, 1], [0, 1, 1]],
                             [[1, 0, 1], [1, 1, 1], [0, 1, 1]]], dtype=float)
        binary = b"triangle test".ljust(80, b"\0") + struct.pack("<I", 2)
        for triangle in vertices:
            binary += struct.pack("<12fH", 0, 0, 1, *triangle.reshape(-1), 0)
        ascii_stl = "solid test\n" + "".join(
            "facet normal 0 0 1\nouter loop\n"
            + "".join("vertex %g %g %g\n" % tuple(p) for p in face)
            + "endloop\nendfacet\n" for face in vertices) + "endsolid test\n"
        with tempfile.TemporaryDirectory() as directory:
            for name, data in (("binary.stl", binary), ("ascii.stl", ascii_stl.encode())):
                path = Path(directory) / name
                path.write_bytes(data)
                unique, faces = projection.load_stl_mesh(path)
                self.assertEqual(unique.shape, (4, 3))
                np.testing.assert_array_equal(unique[faces], vertices)
                self.assertLessEqual(len(projection.load_stl_points(path, 2)), 2)

    def test_union_preserves_concavity_hole_and_overlapping_faces(self):
        # Four rectangular strips form a ring. Interior overlap must not cancel.
        faces = np.concatenate((rectangle(2, 2, 18, 6), rectangle(2, 14, 18, 18),
                                rectangle(2, 2, 6, 18), rectangle(14, 2, 18, 18)))
        mask = projection.triangle_union_mask(np.concatenate((faces, faces)), 22, 22)
        self.assertEqual(mask[4, 4], 255)
        self.assertEqual(mask[10, 4], 255)
        self.assertEqual(mask[10, 10], 0)
        self.assertEqual(mask[0, 0], 0)
        # An L shape is not replaced by its convex hull.
        l_shape = np.concatenate((rectangle(2, 2, 6, 18), rectangle(2, 14, 18, 18)))
        l_mask = projection.triangle_union_mask(l_shape, 22, 22)
        self.assertEqual(l_mask[5, 15], 0)
        self.assertEqual(l_mask[15, 15], 255)

    def test_scanlines_match_independent_barycentric_coverage(self):
        rng = np.random.default_rng(48)
        triangles = rng.uniform(-12, 45, (35, 3, 2))
        y, x = np.indices((32, 32))
        expected = np.zeros((32, 32), bool)
        for a, b, c in triangles:
            e0 = (x - a[0]) * (b[1] - a[1]) - (y - a[1]) * (b[0] - a[0])
            e1 = (x - b[0]) * (c[1] - b[1]) - (y - b[1]) * (c[0] - b[0])
            e2 = (x - c[0]) * (a[1] - c[1]) - (y - c[1]) * (a[0] - c[0])
            expected |= ((e0 >= 0) & (e1 >= 0) & (e2 >= 0)) | ((e0 <= 0) & (e1 <= 0) & (e2 <= 0))
        actual = projection.triangle_union_mask(triangles, 32, 32) != 0
        np.testing.assert_array_equal(actual, expected)

    def test_large_triangle_covers_image_with_all_vertices_outside(self):
        triangle = np.array([[[-1e8, -1e8], [3e8, -1e8], [-1e8, 3e8]]])
        mask = projection.triangle_union_mask(triangle, 32, 24)
        self.assertTrue(np.all(mask == 255))
        invalid = np.full((1, 3, 2), np.nan)
        self.assertFalse(projection.triangle_union_mask(invalid, 32, 24).any())

    def test_near_plane_clips_crossing_faces(self):
        faces = np.array([[[0, 0, 1], [1, 0, 1], [0, 1, -1]],
                          [[0, 0, -1], [1, 0, -1], [0, 1, -1]],
                          [[0, 0, 1], [1, 0, 1], [0, 1, 1]]], dtype=float)
        clipped = projection.clip_triangles_near(faces, near=0.1)
        self.assertEqual(clipped.shape, (3, 3, 3))  # clipped quad plus intact triangle
        self.assertTrue(np.isfinite(clipped).all())
        self.assertTrue(np.all(clipped[:, :, 2] >= 0.1))
        np.testing.assert_allclose(clipped[0], faces[2])


@unittest.skipUnless(cv2 is not None, "OpenCV is required for raster projection")
class TriangleProjectionTests(unittest.TestCase):
    def make_model(self, tips=False):
        model = projection.ProjectionModel.__new__(projection.ProjectionModel)
        model.camera_link = "head_camera_base"
        model.root_link = "base_link"
        model.raster_camera = None
        vertices = np.array([[.1, -.2, 1], [.5, -.2, 1], [.1, .2, 1]])
        model.meshes = [("arm", vertices, np.array([[0, 1, 2]], dtype=np.int32))]
        model.tips = ["zarm_l7_end_effector"] if tips else []
        tip = np.eye(4)
        tip[:3, 3] = [.15, -.1, 1]
        model.fk = lambda q: {"head_camera_base": np.eye(4), "arm": np.eye(4),
                             "zarm_l7_end_effector": tip}
        return model

    def test_renderer_fills_triangle_interior_without_vertex_sampling(self):
        model = self.make_model()
        K = np.array([[110., 0, 87.], [0, 125., 72.], [0, 0, 1.]])
        frame = np.zeros((170, 220, 3), np.uint8)
        result, metrics = model.render(frame, {}, np.eye(4), K, np.zeros(5))
        self.assertGreater(metrics["mask_pixels"], 500)
        self.assertEqual(metrics["projected_triangles"], 1)
        self.assertEqual(metrics["render_mode"], "triangle_union")
        self.assertGreater(result[65, 110, 2], 0)  # interior, far from all vertices

    def test_rotation_flips_only_projection_with_distortion_and_offcentre_intrinsics(self):
        model = self.make_model()
        K = np.array([[110., 0, 87.], [0, 125., 72.], [0, 0, 1.]])
        D = np.array([-.07, .02, .0004, -.0003, 0.])
        rng = np.random.default_rng(23)
        frame = rng.integers(30, 90, (170, 220, 3), dtype=np.uint8)
        plain, _ = model.render(frame, {}, np.eye(4), K, D)
        flipped, _ = model.render(frame, {}, np.eye(4), K, D, pixel_rotation=180)
        mask0 = np.any(plain != frame, axis=2)
        mask180 = np.any(flipped != frame, axis=2)
        np.testing.assert_array_equal(mask180, mask0[::-1, ::-1])
        np.testing.assert_array_equal(flipped[~mask180], frame[~mask180])
        # Separate TCP check avoids rotating label text with the mask.
        model.tips = ["zarm_l7_end_effector"]
        plain, _ = model.render(frame, {}, np.eye(4), K, D)
        flipped, _ = model.render(frame, {}, np.eye(4), K, D, pixel_rotation=180)
        u, v = projection.project_points(np.array([[.15, -.1, 1.]]), K, D,
                                         "plumb_bob", 220, 170)[0]
        np.testing.assert_array_equal(plain[v, u], [255, 100, 0])
        np.testing.assert_array_equal(flipped[169 - v, 219 - u], [255, 100, 0])

    def test_inverse_distortion_does_not_fold_remote_geometry_into_view(self):
        K = np.array([[100., 0, 100.], [0, 100., 100.], [0, 0, 1.]])
        D = np.array([-1., 0, 0, 0, 0])
        camera = projection.RasterCamera(K, D, "plumb_bob", 200, 200)
        # Polynomial forward distortion maps x=1 back near the centre, despite
        # being outside the invertible central camera field.
        xyz = np.array([[1., 0., 1.], [1.05, 0., 1.], [1., .04, 1.]])
        direct = camera.distort(xyz)
        self.assertTrue(np.all((direct > 0) & (direct < 200)))
        mask = projection.triangle_union_mask(camera.ideal_pixels(xyz)[None],
                                               camera.canvas_width, camera.canvas_height)
        self.assertFalse(camera.raw_mask(mask).any())

    def test_fisheye_inverse_map_and_mask_are_finite(self):
        K = np.array([[100., 0, 85.], [0, 105., 60.], [0, 0, 1.]])
        camera = projection.RasterCamera(K, np.array([.01, -.002, 0., 0.]),
                                          "equidistant", 170, 120)
        mask = projection.triangle_union_mask(
            camera.ideal_pixels(np.array([[-.2, -.2, 1.], [.2, -.2, 1.], [0, .2, 1.]]))[None],
            camera.canvas_width, camera.canvas_height)
        actual = camera.raw_mask(mask)
        self.assertEqual(actual.shape, (120, 170))
        self.assertGreater(np.count_nonzero(actual), 500)


if __name__ == "__main__":
    unittest.main()
