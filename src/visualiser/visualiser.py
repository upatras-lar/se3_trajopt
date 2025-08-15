import numpy as np

import pinocchio as pin

from pinocchio.visualize import MeshcatVisualizer
import meshcat.geometry as g
import meshcat.transformations as tf


class TrajoptVisualiser:
    def __init__(self, robot_wrapper):
        self.vis = MeshcatVisualizer(
            robot_wrapper.model,
            robot_wrapper.collision_model,
            robot_wrapper.visual_model,
        )
        self.vis.initViewer(open=True)
        self.vis.loadViewerModel("pinocchio")

    def display_robot_q(self, robot_wrapper, q):
        robot_wrapper.fk_all(q)
        self.vis.display(q)

    def load_terrain(self, terrain):
        self.vis.viewer["terrain"].delete()
        x_range = np.linspace(terrain.min_x, terrain.max_x, terrain.rows)
        y_range = np.linspace(terrain.min_y, terrain.max_y, terrain.cols)
        box_x = np.abs(x_range[0] - x_range[1])
        box_y = np.abs(y_range[0] - y_range[1])

        # Show grid heights above 0.0
        for i, x in enumerate(x_range):
            for j, y in enumerate(y_range):
                height = terrain.grid[i, j]
                if height <= 0:
                    continue
                box = g.Box([box_x, box_y, height])
                translation = [x, y, height / 2.0]
                transform = tf.translation_matrix(translation)
                self.vis.viewer["terrain"][f"cell_{i}_{j}"].set_object(box)
                self.vis.viewer["terrain"][f"cell_{i}_{j}"].set_transform(transform)

    def update_forces(self, robot_wrapper, forces_dict, scale=1):
        self.vis.viewer["forces"].delete()
        for fid, f_F in forces_dict.items():
            # Convert force vector to float (to avoid dtype issues)
            force_arrow = scale * f_F

            fid = robot_wrapper.model.getFrameId(fid)

            # Get world position of the frame
            oMF = robot_wrapper.data.oMf[fid]
            force_start = oMF.translation  # Origin of force
            force_end = force_start + oMF.rotation @ force_arrow  # Force direction

            # Create vertices for the line
            verts = np.vstack(
                (force_start, force_end)
            ).T  # Transpose to get the correct shape

            # Create the line geometry
            line_geometry = g.Line(
                g.PointsGeometry(verts),
                g.LineBasicMaterial(color=0xFF0000, linewidth=5),
            )

            # Set the force geometry in the visualizer
            self.vis.viewer["forces"][f"force_{fid}"].set_object(line_geometry)
