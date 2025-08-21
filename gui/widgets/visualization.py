import math
import numpy as np
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.pyplot import figure

class Graphic3d(FigureCanvas):
    def __init__(self):
        self.plot_figure = figure(figsize=(5, 7))
        self.ax = self.plot_figure.add_subplot(111, projection='3d')
        super().__init__(self.plot_figure)
        
        self.BASE_RADIUS = 100
        self.PAYLOAD_RADIUS = 100
        self.ACTUATOR_LAYOUT_RADIUS = 80
        self.ACTUATOR_RADIUS = 20

        self.show_neutral_pose()

    def data_for_disc_along_z(self, center_x, center_y, radius):
        z = np.linspace(0, 0, 50)
        theta = np.linspace(0, 2*np.pi, 50)
        radius = np.linspace(0, radius, 50)
        theta_grid, radius_grid = np.meshgrid(theta, radius)
        _, z_grid = np.meshgrid(theta, z)

        x_grid = radius_grid * np.cos(theta_grid) + center_x
        y_grid = radius_grid * np.sin(theta_grid) + center_y
        return x_grid, y_grid, z_grid

    def data_for_torus(self, start_point, radius, radius_of_curvature, phi, theta):
        n = 100
        big_theta = theta
        theta_vals = np.linspace(0, 2.*np.pi, n)
        phi_vals = np.linspace(0, phi, n)
        theta_grid, phi_grid = np.meshgrid(theta_vals, phi_vals)

        R0, a = radius_of_curvature, radius

        x = (R0 + a*np.cos(theta_grid)) * np.cos(phi_grid) - radius_of_curvature
        z = (R0 + a*np.cos(theta_grid)) * np.sin(phi_grid)
        y = a * np.sin(theta_grid)

        x, y, z = self.transform_grid(x, y, z, [[start_point[0]], [start_point[1]], [start_point[2]], [math.pi + big_theta], [0], [0]])
        return x, y, z

    def transform_grid(self, Xc, Yc, Zc, a):
        alpha, beta, gamma = a[3][0], a[4][0], a[5][0]
        R = np.array([
            [math.cos(alpha)*math.cos(beta), math.cos(alpha)*math.sin(beta)*math.sin(gamma)-math.sin(alpha)*math.cos(gamma), math.cos(alpha)*math.sin(beta)*math.cos(gamma)+math.sin(alpha)*math.sin(gamma)],
            [math.sin(alpha)*math.cos(beta), math.sin(alpha)*math.sin(beta)*math.sin(gamma)+math.cos(alpha)*math.cos(gamma), math.sin(alpha)*math.sin(beta)*math.cos(gamma)-math.cos(alpha)*math.sin(gamma)],
            [-math.sin(beta),         math.cos(beta)*math.sin(gamma),                          math.cos(beta)*math.cos(gamma)]
        ])

        for i in range(len(Xc)):
            for j in range(len(Xc[i])):
                point = [[Xc[i][j]], [Yc[i][j]], [Zc[i][j]]]
                new_point = np.dot(R, point)
                Xc[i][j] = new_point[0][0] + a[0][0]
                Yc[i][j] = new_point[1][0] + a[1][0]
                Zc[i][j] = new_point[2][0] + a[2][0]
        return Xc, Yc, Zc

    def transform(self, p, a):
        alpha, beta, gamma = a[3][0], a[4][0], a[5][0]
        R = np.array([
            [math.cos(alpha)*math.cos(beta), math.cos(alpha)*math.sin(beta)*math.sin(gamma)-math.sin(alpha)*math.cos(gamma), math.cos(alpha)*math.sin(beta)*math.cos(gamma)+math.sin(alpha)*math.sin(gamma)],
            [math.sin(alpha)*math.cos(beta), math.sin(alpha)*math.sin(beta)*math.sin(gamma)+math.cos(alpha)*math.cos(gamma), math.sin(alpha)*math.sin(beta)*math.cos(gamma)-math.cos(alpha)*math.sin(gamma)],
            [-math.sin(beta),         math.cos(beta)*math.sin(gamma),                          math.cos(beta)*math.cos(gamma)]
        ])
        p = np.dot(R, p)
        p[0] += a[0][0]
        p[1] += a[1][0]
        p[2] += a[2][0]
        return p

    def update_pose(self, a, radius_of_curvature, theta, phi):
        self.ax.clear()

        Xb, Yb, Zb = self.data_for_disc_along_z(0, 0, self.BASE_RADIUS)
        Xp, Yp, Zp = self.data_for_disc_along_z(0, 0, self.PAYLOAD_RADIUS)
        Xp, Yp, Zp = self.transform_grid(Xp, Yp, Zp, a)

        curve_direction = [math.cos(theta), math.sin(theta), 0]
        actuator_surfaces = []
        for i in range(3):
            actuator_point = [self.ACTUATOR_LAYOUT_RADIUS*math.cos(math.radians(120*i)),
                              self.ACTUATOR_LAYOUT_RADIUS*math.sin(math.radians(120*i)), 0]
            new_curve_radius = radius_of_curvature - np.dot(actuator_point, curve_direction)
            actuator_surfaces.append(self.data_for_torus(actuator_point, self.ACTUATOR_RADIUS, new_curve_radius, phi, theta))

        self.ax.set_proj_type('ortho')
        self.ax.set_xlim([-100, 100])
        self.ax.set_ylim([-100, 100])
        self.ax.set_zlim([0, 200])
        self.ax.plot_surface(Xb, Yb, Zb, alpha=0.5)
        self.ax.plot_surface(Xp, Yp, Zp, alpha=0.5)
        for surf in actuator_surfaces:
            self.ax.plot_surface(surf[0], surf[1], surf[2], alpha=0.5)

        x_marker_start = self.transform([0, 0, 0], a)
        x_marker_end = self.transform([self.PAYLOAD_RADIUS, 0, 0], a)
        self.ax.plot([x_marker_start[0], x_marker_end[0]],
                     [x_marker_start[1], x_marker_end[1]],
                     [x_marker_start[2], x_marker_end[2]])
        self.draw()

    def show_neutral_pose(self):
        self.update_pose([[0], [0], [150], [0], [0], [0]], 180000000000, 0, 8.333333333333332e-10)

    def update_pose_from_list(self, pose_list):
        """
        Update the 3D visualization based on a pose list from the high-level processor.
        
        Args:
            pose_list (list): [x, y, z, roll, pitch, yaw] - position and orientation
        """
        if not pose_list or len(pose_list) < 6:
            return
            
        # Convert pose list to the format expected by update_pose
        x, y, z, roll, pitch, yaw = pose_list
        a = [[x], [y], [z], [yaw], [pitch], [roll]]  # Note the different order of rotations
        
        # For simplicity, using default parameters for radius_of_curvature, theta, phi
        radius_of_curvature = 180000000000
        theta = 0
        phi = 8.333333333333332e-10
        
        # Update the visualization
        self.update_pose(a, radius_of_curvature, theta, phi) 