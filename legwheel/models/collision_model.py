import numpy as np
from legwheel.models.corgi_robot import CorgiRobot
from legwheel.config import RobotParams


class CorgiCollisionModel:
    """
    A kinematic collision model for the Corgi robot to analyze self-righting strategies.
    It computes the 3D positions of key bounding points (chassis corners, M6 studs, wheel bottoms)
    in the World Frame {W} to determine which points are in contact with the ground.
    """

    def __init__(self, robot: CorgiRobot):
        self.robot = robot

        # 1. Base Chassis (Octagonal Prism) Local Points
        self.chassis_local_points = self._generate_chassis_points()

        # 2. Grab Points (Customizable later)
        # e.g. {"top_handle": [x, y, z], "bottom_handle": [x, y, z]}
        self.grab_local_points = {}

    def _generate_chassis_points(self):
        """Generates the 16 corner points of the octagonal prism chassis in Body Frame {B}."""
        l = RobotParams.CHASSIS_LENGTH
        w = RobotParams.CHASSIS_WIDTH
        h = RobotParams.CHASSIS_HEIGHT
        z_chassis = RobotParams.ABAD_AXIS_OFFSET
        c = 0.04  # Chamfer distance (40 mm)

        # Cross section points in Y-Z plane
        y_points = np.array(
            [w / 2 - c, w / 2, w / 2, w / 2 - c, -w / 2 + c, -w / 2, -w / 2, -w / 2 + c]
        )
        z_points = (
            np.array([h / 2, h / 2 - c, -h / 2 + c, -h / 2, -h / 2, -h / 2 + c, h / 2 - c, h / 2])
            + z_chassis
        )

        points = []
        # Front face
        for y, z in zip(y_points, z_points):
            points.append([l / 2, y, z])
        # Back face
        for y, z in zip(y_points, z_points):
            points.append([-l / 2, y, z])

        return np.array(points)

    def get_m6_stud_points(self, q_list):
        """
        Calculates the 4 M6 stud points (one on each wheel's outer side) in World Frame {W}.
        The M6 stud is located at x_M = d_wheel + 34.5mm in the Module Frame {Mi}.
        """
        stud_points_W = []
        d_wheel = RobotParams.WHEEL_AXIAL_OFFSET
        m6_offset = 0.0345  # 34.5 mm

        for i, q in enumerate(q_list):
            leg = self.robot.legs[i]
            gamma = q[2]

            # The stud is along the X axis of {Mi}
            # For left legs (0, 3), {Mi} X axis points outward (+Y in general).
            # We use the leg's internal mapping
            p_M = np.array([d_wheel + m6_offset, 0, 0])

            p_B = leg._M_to_B(p_M, gamma)
            p_W = self.robot.body_to_world(p_B)
            stud_points_W.append(p_W)

        return np.array(stud_points_W)

    def get_wheel_contact_points(self, q_list):
        """
        Calculates the lowest point of each wheel in World Frame {W}.
        Uses 10-degree sampling around the rim to robustly find the true lowest point
        regardless of body orientation.
        """
        wheel_points_W = []
        for i, q in enumerate(q_list):
            theta, beta, gamma = q
            leg = self.robot.legs[i]

            min_z = float("inf")
            lowest_p_W = None

            # Sample every 10 degrees around the rim
            for alpha in np.arange(-180, 180, 10):
                # Check both inner and outer edges of the wheel thickness
                for w in [leg.wheel_thickness / 2.0, -leg.wheel_thickness / 2.0]:
                    p_B = leg.forward_kinematics(theta, beta, gamma, alpha, w)
                    p_W = self.robot.body_to_world(p_B)
                    if p_W[2] < min_z:
                        min_z = p_W[2]
                        lowest_p_W = p_W

            wheel_points_W.append(lowest_p_W)

        return np.array(wheel_points_W)

    def get_chassis_points_W(self):
        """Returns the chassis corner points in World Frame {W}."""
        points_W = []
        for p_B in self.chassis_local_points:
            points_W.append(self.robot.body_to_world(p_B))
        return np.array(points_W)

    def get_all_collision_points(self, q_list):
        """
        Returns a dictionary of all key collision points in World Frame {W}.
        """
        return {
            "chassis": self.get_chassis_points_W(),
            "m6_studs": self.get_m6_stud_points(q_list),
            "wheels": self.get_wheel_contact_points(q_list),
        }

    def find_contact_pivots(self, q_list, tolerance=1e-3):
        """
        Finds the points with the lowest Z coordinate in the World Frame.
        Returns the minimum Z value and the list of points that are at this height.
        """
        all_points = self.get_all_collision_points(q_list)

        # Flatten all points
        labeled_points = []
        for i, p in enumerate(all_points["chassis"]):
            labeled_points.append({"label": f"Chassis_P{i}", "pos": p})

        for i, p in enumerate(all_points["m6_studs"]):
            labeled_points.append({"label": f"M6_Stud_Leg{i}", "pos": p})

        for i, p in enumerate(all_points["wheels"]):
            labeled_points.append({"label": f"Wheel_Leg{i}", "pos": p})

        # Find minimum Z
        min_z = min(p["pos"][2] for p in labeled_points)

        # Filter points near min_z
        contact_points = [p for p in labeled_points if p["pos"][2] <= min_z + tolerance]

        return min_z, contact_points
