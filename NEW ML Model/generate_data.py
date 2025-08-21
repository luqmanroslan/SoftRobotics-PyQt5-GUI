# @title Stewart Platform Class
import numpy as np
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
import math
from math import sin as s
from math import cos as c

class StewartPlatform:
    def __init__(self, r_b , phi_b , r_p, phi_p):
        self.r_b = r_b # radius of base
        self.phi_b = phi_b # angle between base joints
        self.r_p = r_p # radius of platform
        self.phi_p = phi_p # angle between platform joints
        self.options = {
            1: self.getSingularValueIndex,
            2: self.getManipulabilityIndex,
            3: self.getConditionNumber,
            4: self.getLocalConditionIndex,
            5: self.getLDI,
            6: self.getLocalConditionIndexT
        }


    ############################################ IK ##############################################
    def __inverse_kinematics(self, pose): ## Inverse Kinematics (Vectorial Closing Loop)
        if len(pose) != 6:
            raise ValueError("Pose must be a 6-element list or array")

        self.p_i = np.zeros([6, 3])
        self.b_i = np.zeros([6, 3])
        self.l_i = np.zeros([6, 3])
        self.l_i_unit = np.zeros([6, 3])

        self.pose_vect = pose
        x_p, y_p, z_p, roll, pitch, yaw = pose

        self.r = R.from_euler("ZYX", (yaw, pitch, roll), degrees=True)  #rotation matrix
        angles = np.array([0, 60, 60, 180, 180, 0])

        for i in range(6):
            j = -1 if (i % 2) == 1 else 1
            self.b_i[i, 0] = self.r_b * np.cos(np.deg2rad(angles[i] + j * self.phi_b / 2)) # base points
            self.b_i[i, 1] = self.r_b * np.sin(np.deg2rad(angles[i] + j * self.phi_b / 2))
            self.p_i[i, 0] = self.r_p * np.cos(np.deg2rad(angles[i] + j * self.phi_p / 2)) # platform points
            self.p_i[i, 1] = self.r_p * np.sin(np.deg2rad(angles[i] + j * self.phi_p / 2))
            self.l_i[i, :] = np.array([x_p, y_p, z_p]) + self.r.as_matrix().dot(self.p_i[i]) - self.b_i[i] # legs vectors
            self.l_i_unit[i, :] = self.l_i[i, :] / np.linalg.norm(self.l_i[i, :]) # unit legs vectors
            self.p_i[i] = self.l_i[i, :] + self.b_i[i] # moving platform points wrt base frame

        return self.l_i

    def getIK(self, pose):
        return self.__inverse_kinematics(pose)

    ############################################ Jacobian ##############################################
    def __jacobian(self): # Calculate Jacobian q_dot=Jacobian*x_dot
        if not hasattr(self, 'l_i_unit') or not hasattr(self, 'p_i'):
            raise AttributeError("Run inverse kinematics before calculating the Jacobian")
        self.jacobian = np.zeros([6, 6])
        for i in range(6):
            lastcols = np.cross(self.r.as_matrix().dot(self.p_i[i]), self.l_i_unit[i])
            self.jacobian[i, :3] = self.l_i_unit[i]
            self.jacobian[i, 3:] = lastcols

        return self.jacobian

    def getJacobian(self):
        return self.__jacobian()

    ############################################ FK ##############################################
    def __forward_kinematics(self, starting_pose, lengths_desired, plot_flag): # Newton Method Optimization.
        if not isinstance(starting_pose, (list, np.ndarray)) or len(starting_pose) != 6:
            raise ValueError("Starting pose must be a list or numpy array with 6 elements")
        if not isinstance(lengths_desired, (list, np.ndarray)) or len(lengths_desired) != 6:
            raise ValueError("Lengths desired must be a list or numpy array with 6 elements")

        self.pose_vect = starting_pose
        x_p, y_p, z_p, roll, pitch, yaw = starting_pose
        self.r = R.from_euler("ZYX", (yaw, pitch, roll), degrees=True)

        # Newton Method Parameters
        max_count = 100
        epsilon = 0.1#0.0001
        alpha_pos = 0.2
        alpha_rot = 0.5
        # variables
        est_pose_vect = np.copy(starting_pose)
        est_pose = np.zeros([4, 4])
        est_pose[:3, :3] = self.r.as_matrix()
        est_pose[:3, 3] = [x_p, y_p, z_p]
        est_pose[3, 3] = 1
        delta_lengths = np.zeros(6)
        delta_T = np.identity(4)
        error = epsilon + 1
        count = 0

        while error > epsilon and count < max_count:
            # find new variables
            legs_est = self.__inverse_kinematics(est_pose_vect)
            lengths_est = np.linalg.norm(legs_est, axis=1)
            delta_lengths = lengths_desired - lengths_est
            J = self.__jacobian()
            J_T = J.transpose()
            # find x_delta with pseudoinverse
            J_star = np.linalg.inv(J_T.dot(J) + 0.0001 * np.identity(6)).dot(J_T)
            x_delta = np.dot(J_star, delta_lengths)
            x_delta_pos = x_delta[:3] * alpha_pos
            x_delta_rot = x_delta[3:] * alpha_rot
            # transform x_delta to delta_T
            r_delta = R.from_euler("ZYX", x_delta_rot)
            delta_T[:3, :3] = r_delta.as_matrix()
            delta_T[:3, 3] = x_delta[:3]
            # move pose
            est_pose = np.dot(delta_T, est_pose)
            # transform est_pose to est_pose_vect
            est_pose_vect[:3] = est_pose[:3, 3]
            self.r_pos = R.from_matrix(est_pose[:3, :3])
            est_pose_vect[3:] = self.r_pos.as_euler("ZYX", degrees=True)
            # find error and update counter
            error = np.linalg.norm(delta_lengths)
            count += 1

            # Uncomment to plot during FK procedure
            if plot_flag:
              self.plot()

        if count < max_count:
            self.pose_vect = est_pose_vect
            """print("Forward Kinematics converged!")
            print("Estimated pose:", est_pose_vect)
            print("Desired Lengths:", lengths_desired)
            print("Estimated Lengths:", lengths_est)
            print("Error:", error)
            print("Iterations:", count)"""
        else:
            """print("Forward Kinematics did not converge")
            print("Estimated pose:", est_pose_vect)
            print("Desired Lengths:", lengths_desired)
            print("Estimated Lengths:", lengths_est)
            print("Error:", error)
            print("Iterations:", count)"""

        return self.pose_vect

    def getFK(self, starting_pose, lengths_desired, plot_flag):
        return self.__forward_kinematics(starting_pose, lengths_desired, plot_flag)

    ############################################ Kinematic Analysis ##############################################
    def __find_singular_value_index(self): # measures drive capability of the platform, finds max q_dot under unitary x_dot
        J = self.__jacobian()
        eigenvalues = np.linalg.eigvals(J.T.dot(np.linalg.inv(J)))
        sigma_max = np.abs(np.sqrt(np.max(eigenvalues)))
        return sigma_max

    def getSingularValueIndex(self):
        return self.__find_singular_value_index()

    def __find_manipulability_index(self): # Measures manipulability of manipulator, can be used to optimize it's configuration
        J = self.__jacobian()
        JJ = J.dot(J.T)
        w = np.sqrt(np.linalg.det(JJ))
        return w

    def getManipulabilityIndex(self):
        return self.__find_manipulability_index()

    def __find_condition_number(self): # Measures closeness to isotropic configuration [1,+ inf)
        J = self.__jacobian()
        cond_num = np.linalg.cond(J)
        return cond_num

    def getConditionNumber(self):
        return self.__find_condition_number()

    def __find_local_condition_index(self): # Measures closeness to isotropic configuration (0,1]
        eta = 1 / self.__find_condition_number()
        return eta

    def getLocalConditionIndex(self):
        return self.__find_local_condition_index()

    ############################################ Force Analysis ##############################################
    def __find_platform_forces(self, F_actuators): # Finds platform forces given by actuator forces
        if not isinstance(F_actuators, (list, np.ndarray)) or len(F_actuators) != 6:
            raise ValueError("F_actuators must be a list or numpy array with 6 elements")
        J_T = self.__jacobian().T
        F_platform = J_T.dot(F_actuators)
        return F_platform

    def getPlatformForces(self, F_actuators):
        return self.__find_platform_forces(F_actuators)

    def __find_actuator_forces(self, F_platform): # Finds actuator forces given by platform forces
        if not isinstance(F_platform, (list, np.ndarray)) or len(F_platform) != 6:
            raise ValueError("F_platform must be a list or numpy array with 6 elements")

        J_Inv = np.linalg.inv(self.__jacobian().T) # only when Jacobian is invertible
        F_actuators = J_Inv.dot(F_platform)
        return F_actuators

    def getActuatorForces(self, F_platform):
        return self.__find_actuator_forces(F_platform)

    def __find_force_ellipsoid(self): # Finds force ellipsoid (excluding moment vectors)
        J_T = self.__jacobian().T
        JJ = J_T.dot(J_T.T)
        A = JJ[:3, :3] # Lagrangian
        eigenvalues, eigenvectors = np.linalg.eig(A)

        Force_ellipsoid_1 = eigenvectors[:, 0] * np.abs(eigenvalues[0])
        Force_ellipsoid_2 = eigenvectors[:, 1] * np.abs(eigenvalues[1])
        Force_ellipsoid_3 = eigenvectors[:, 2] * np.abs(eigenvalues[2])

        return [Force_ellipsoid_1, Force_ellipsoid_2, Force_ellipsoid_3]

    def getForceEllipsoid(self):
        return self.__find_force_ellipsoid()

    def __find_ldi(self):  # Local design index for Force transmittability (actuator design)
        J_T = self.__jacobian().T
        JJ = J_T.dot(J_T.T)
        A = JJ[:3, :3] # Lagrangian
        eigenvalues, _ = np.linalg.eig(A)
        R_f = np.abs(np.sqrt(np.min(eigenvalues))) # take minimum magnitude of force ellipsoid for actuator design
        LDI_f = 1 / R_f # local design index
        return LDI_f

    def getLDI(self):
        return self.__find_ldi()

    def __find_local_condition_index_T(self):  # Measures closeness to isotropic force configuration (0,1] 0 is singularity.
      J_T = self.__jacobian().T
      cond_num = np.linalg.cond(J_T)
      eta_T = 1 / cond_num
      return eta_T

    def getLocalConditionIndexT(self):
      return self.__find_local_condition_index_T()

    ############################################ Workspace Analysis #########################################

    def __find_index_workspace_position(self, workspace_limits, RPY, N, choice):
        # Validate inputs
        if not isinstance(workspace_limits, (list, np.ndarray)) or len(workspace_limits) != 6:
            raise ValueError("workspace_limits must be a list or array of 6 elements")
        if not isinstance(RPY, (list, np.ndarray)) or len(RPY) != 3:
            raise ValueError("RPY must be a list or array of 3 elements")
        if not isinstance(N, int) or N <= 0:
            raise ValueError("N must be a positive integer")
        if choice not in self.options:
            raise ValueError(f"Invalid choice for index calculation. Valid choices are: {list(self.options.keys())}")

        # Define discretization space
        x_min, x_max, y_min, y_max, z_min, z_max = workspace_limits
        roll, pitch, yaw = RPY

        # Ensure workspace limits are valid
        if x_min >= x_max or y_min >= y_max or z_min >= z_max:
            raise ValueError("Workspace limits must define a valid range (min < max) for each dimension")

        # Discretization vectors
        try:
            x_vect = np.linspace(x_min, x_max, N)
            y_vect = np.linspace(y_min, y_max, N)
            z_vect = np.linspace(z_min, z_max, N)
        except Exception as e:
            raise ValueError(f"Error in creating discretization vectors: {e}")

        xx, yy, zz = np.meshgrid(x_vect, y_vect, z_vect, indexing='ij')
        positions = np.vstack([xx.ravel(), yy.ravel(), zz.ravel()]).T
        orientations = np.array([roll, pitch, yaw])

        Holder = []

        for pos in positions:
            p_vect = np.hstack((pos, orientations))
            try:
                self.getIK(p_vect)
                Index = self.options[choice]()
                Holder.append(np.append(pos, Index))
            except Exception as e:
                print(f"Error in calculating index for position {pos}: {e}")
                continue

        if not Holder:
            raise RuntimeError("No valid indices calculated. Check the input parameters and IK method.")

        Holder = np.array(Holder)
        return Holder

    def getIndexWorkspacePosition(self, workspace_limits, RPY, N, choice):
        return self.__find_index_workspace_position(workspace_limits, RPY, N, choice)

    def __find_index_workspace_orientation(self, position, orientation_limits, N, choice):
        # Validate inputs
        if not isinstance(position, (list, np.ndarray)) or len(position) != 3:
            raise ValueError("position must be a list or array of 3 elements")
        if not isinstance(orientation_limits, (list, np.ndarray)) or len(orientation_limits) != 6:
            raise ValueError("orientation_limits must be a list or array of 6 elements")
        if not isinstance(N, int) or N <= 0:
            raise ValueError("N must be a positive integer")
        if choice not in self.options:
            raise ValueError(f"Invalid choice for index calculation. Valid choices are: {list(self.options.keys())}")

        # Define discretization space
        roll_min, roll_max, pitch_min, pitch_max, yaw_min, yaw_max = orientation_limits

        # Ensure orientation limits are valid
        if roll_min >= roll_max or pitch_min >= pitch_max or yaw_min >= yaw_max:
            raise ValueError("Orientation limits must define a valid range (min < max) for each dimension")

        # Discretization vectors
        try:
            roll_vect = np.linspace(roll_min, roll_max, N)
            pitch_vect = np.linspace(pitch_min, pitch_max, N)
            yaw_vect = np.linspace(yaw_min, yaw_max, N)
        except Exception as e:
            raise ValueError(f"Error in creating discretization vectors: {e}")

        rr, pp, yy = np.meshgrid(roll_vect, pitch_vect, yaw_vect, indexing='ij')
        orientations = np.vstack([rr.ravel(), pp.ravel(), yy.ravel()]).T

        Holder = []

        for orient in orientations:
            p_vect = np.hstack((position, orient))
            try:
                self.getIK(p_vect)
                Index = self.options[choice]()
                Holder.append(np.append(orient, Index))
            except Exception as e:
                print(f"Error in calculating index for orientation {orient}: {e}")
                continue

        if not Holder:
            raise RuntimeError("No valid indices calculated. Check the input parameters and IK method.")

        Holder = np.array(Holder)
        return Holder

    def getIndexWorkspaceOrientation(self, position, orientation_limits, N, choice):
        return self.__find_index_workspace_orientation(position, orientation_limits, N, choice)

    def __find_index_workspace(self, workspace_limits, orientation_limits, N_pos, N_orient, choice):
        # Validate inputs
        if not isinstance(workspace_limits, (list, np.ndarray)) or len(workspace_limits) != 6:
            raise ValueError("workspace_limits must be a list or array of 6 elements")
        if not isinstance(orientation_limits, (list, np.ndarray)) or len(orientation_limits) != 6:
            raise ValueError("orientation_limits must be a list or array of 6 elements")
        if not isinstance(N_pos, int) or N_pos <= 0:
            raise ValueError("N_pos must be a positive integer")
        if not isinstance(N_orient, int) or N_orient <= 0:
            raise ValueError("N_orient must be a positive integer")
        if choice not in self.options:
            raise ValueError(f"Invalid choice for index calculation. Valid choices are: {list(self.options.keys())}")

        # Define discretization space for positions
        x_min, x_max, y_min, y_max, z_min, z_max = workspace_limits
        if x_min >= x_max or y_min >= y_max or z_min >= z_max:
            raise ValueError("Workspace limits must define a valid range (min < max) for each dimension")

        x_vect = np.linspace(x_min, x_max, N_pos)
        y_vect = np.linspace(y_min, y_max, N_pos)
        z_vect = np.linspace(z_min, z_max, N_pos)

        xx, yy, zz = np.meshgrid(x_vect, y_vect, z_vect, indexing='ij')
        positions = np.vstack([xx.ravel(), yy.ravel(), zz.ravel()]).T

        # Define discretization space for orientations
        roll_min, roll_max, pitch_min, pitch_max, yaw_min, yaw_max = orientation_limits
        if roll_min >= roll_max or pitch_min >= pitch_max or yaw_min >= yaw_max:
            raise ValueError("Orientation limits must define a valid range (min < max) for each dimension")

        roll_vect = np.linspace(roll_min, roll_max, N_orient)
        pitch_vect = np.linspace(pitch_min, pitch_max, N_orient)
        yaw_vect = np.linspace(yaw_min, yaw_max, N_orient)

        rr, pp, yy = np.meshgrid(roll_vect, pitch_vect, yaw_vect, indexing='ij')
        orientations = np.vstack([rr.ravel(), pp.ravel(), yy.ravel()]).T

        # Iterate through each position and each orientation
        Holder = []
        for pos in positions:
            for orient in orientations:
                p_vect = np.hstack((pos, orient))
                try:
                    self.getIK(p_vect)
                    Index = self.options[choice]()
                    Holder.append(np.append(p_vect, Index))
                except Exception as e:
                    print(f"Error in calculating index for position {pos} and orientation {orient}: {e}")
                    continue

        if not Holder:
            raise RuntimeError("No valid indices calculated. Check the input parameters and IK method.")

        Holder = np.array(Holder)
        return Holder

    def getIndexWorkspace(self, workspace_limits, orientation_limits, N_pos, N_orient, choice):
        return self.__find_index_workspace(workspace_limits, orientation_limits, N_pos, N_orient, choice)

    def __find_singularity_workspace(self, workspace_limits, orientation_limits, N_pos, N_orient):
        # Validate inputs
        if not isinstance(workspace_limits, (list, np.ndarray)) or len(workspace_limits) != 6:
            raise ValueError("workspace_limits must be a list or array of 6 elements")
        if not isinstance(orientation_limits, (list, np.ndarray)) or len(orientation_limits) != 6:
            raise ValueError("orientation_limits must be a list or array of 6 elements")
        if not isinstance(N_pos, int) or N_pos <= 0:
            raise ValueError("N_pos must be a positive integer")
        if not isinstance(N_orient, int) or N_orient <= 0:
            raise ValueError("N_orient must be a positive integer")


        # Define discretization space for positions
        x_min, x_max, y_min, y_max, z_min, z_max = workspace_limits
        if x_min >= x_max or y_min >= y_max or z_min >= z_max:
            raise ValueError("Workspace limits must define a valid range (min < max) for each dimension")

        x_vect = np.linspace(x_min, x_max, N_pos)
        y_vect = np.linspace(y_min, y_max, N_pos)
        z_vect = np.linspace(z_min, z_max, N_pos)

        xx, yy, zz = np.meshgrid(x_vect, y_vect, z_vect, indexing='ij')
        positions = np.vstack([xx.ravel(), yy.ravel(), zz.ravel()]).T

        # Define discretization space for orientations
        roll_min, roll_max, pitch_min, pitch_max, yaw_min, yaw_max = orientation_limits
        if roll_min >= roll_max or pitch_min >= pitch_max or yaw_min >= yaw_max:
            raise ValueError("Orientation limits must define a valid range (min < max) for each dimension")

        roll_vect = np.linspace(roll_min, roll_max, N_orient)
        pitch_vect = np.linspace(pitch_min, pitch_max, N_orient)
        yaw_vect = np.linspace(yaw_min, yaw_max, N_orient)

        rr, pp, yy = np.meshgrid(roll_vect, pitch_vect, yaw_vect, indexing='ij')
        orientations = np.vstack([rr.ravel(), pp.ravel(), yy.ravel()]).T

        # Select 1/condition number for J_T
        choice=6

        # Iterate through each position and each orientation
        Holder = []
        for pos in positions:
            for orient in orientations:
                p_vect = np.hstack((pos, orient))
                try:
                    self.getIK(p_vect)
                    Index = self.options[choice]()
                    if Index <0.001:
                      Holder.append(np.append(p_vect, Index))
                except Exception as e:
                    print(f"Error in calculating index for position {pos} and orientation {orient}: {e}")
                    continue

        if not Holder:
            raise RuntimeError("No valid indices calculated. Check the input parameters and IK method.")

        Holder = np.array(Holder)
        return Holder

    def getSingularityWorkspace(self, workspace_limits, orientation_limits, N_pos, N_orient): # returns the value of the local condition indexT for every point in space
        # Choosing N_pos and N_orient too high may result in a computational expensive operation, suggested values ( N_pos=10, N_orient=10 )
        # for practical usage there is the need to filter the data. Suggestion: filter by local condition index value AND by distance between data points (from scipy.spatial.distance import cdist).
        return self.__find_singularity_workspace(workspace_limits, orientation_limits, N_pos, N_orient)

    ############################################ Plot Platform ##############################################
    def plot(self):
        if not hasattr(self, 'p_i'):
          raise AttributeError("Run inverse kinematics before calling the Plot function")
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')
        ax.set_title("Stewart Platform")

        ax.plot(self.p_i[:, 0], self.p_i[:, 1], self.p_i[:, 2], color='b', lw=3)
        ax.plot([self.p_i[5, 0], self.p_i[0, 0]], [self.p_i[5, 1], self.p_i[0, 1]], [self.p_i[5, 2], self.p_i[0, 2]], color='b', lw=3)
        ax.scatter(self.p_i[:, 0], self.p_i[:, 1], self.p_i[:, 2], color='c', marker='o')

        ax.plot(self.p_i[:, 0], self.p_i[:, 1],0, color='k', lw=1)
        ax.plot([self.p_i[5, 0], self.p_i[0, 0]], [self.p_i[5, 1], self.p_i[0, 1]], [0, 0], color='k', lw=1)

        ax.plot(self.b_i[:, 0], self.b_i[:, 1], self.b_i[:, 2], color='b')
        ax.plot([self.b_i[5, 0], self.b_i[0, 0]], [self.b_i[5, 1], self.b_i[0, 1]], [self.b_i[0, 2], self.b_i[0, 2]], color='b')
        ax.scatter(self.b_i[:, 0], self.b_i[:, 1], self.b_i[:, 2], color='c', marker='o')

        for i in range(6):
            ax.plot([self.b_i[i, 0], self.p_i[i, 0]], [self.b_i[i, 1], self.p_i[i, 1]], [self.b_i[i, 2], self.p_i[i, 2]], color='k', lw=3)

        displacement = 0.03
        for x, y, z, i in zip(self.b_i[:, 0], self.b_i[:, 1], self.b_i[:, 2], range(len(self.b_i[:, 0]))):
            ax.text(x + displacement, y + displacement, z, i)

        ax.quiver(0, 0, 0, 1, 0, 0, color='r', lw=0.5, length=0.2)
        ax.quiver(0, 0, 0, 0, 1, 0, color='g', lw=0.5, length=0.2)
        ax.quiver(0, 0, 0, 0, 0, 1, color='b', lw=0.5, length=0.2)

        x_limits = ax.get_xlim3d()
        y_limits = ax.get_ylim3d()
        z_limits = ax.get_zlim3d()
        x_range = abs(x_limits[1] - x_limits[0])
        x_middle = np.mean(x_limits)
        y_range = abs(y_limits[1] - y_limits[0])
        y_middle = np.mean(y_limits)
        z_range = abs(z_limits[1] - z_limits[0])
        z_middle = np.mean(z_limits)
        plot_radius = 0.5 * max([x_range, y_range, z_range])

        ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
        ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
        ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])

        ax.set_xlabel('X-axis')
        ax.set_ylabel('Y-axis')
        ax.set_zlabel('Z-axis')

        plt.show()

class actuator_kinematic_solver:
    def __init__(self, actuator_layout_radius, L0):
        self.ACTUATOR_LAYOUT_RADIUS = actuator_layout_radius
        self.L0 = L0*2

    def forward(self, lengths):
        l1, l2, l3,  = lengths * 2
        curve_r, phi, theta = self.ActuatorToConfig(l1,l2,l3)
        a = self.ConfigToCartesian(curve_r, phi, theta)
    
        return a, curve_r, phi, theta

    def ActuatorToConfig(self, l1, l2, l3):
        x = math.sqrt(l1**2 + l2**2 + l3**2 - l1*l2 - l2*l3 - l1*l3 + 1e-14)

        radius_of_curvature = (3*self.L0 + l1 + l2 + l3)*self.ACTUATOR_LAYOUT_RADIUS / (2*x)
        phi = x/(3*self.ACTUATOR_LAYOUT_RADIUS)
        theta = math.atan2(math.sqrt(3)*(l3 - l2), (l2 + l3 - 2*l1))
        return radius_of_curvature, phi, theta
    
    def ConfigToCartesian(self, radius_of_curvature, phi, theta):

        T1 = np.matrix([[ c(-theta), -s(-theta), 0, -radius_of_curvature],
                       [ s(-theta),  c(-theta), 0, 0],
                       [ 0,0,1,0],
                       [ 0,0,0,1]])
        
        T2 = np.matrix([[ c(phi), 0, s(phi), radius_of_curvature],
                       [ 0, 1, 0, 0],
                       [-s(phi), 0, c(phi), 0],
                       [ 0,0,0,1]])
        
        T3 = np.matrix([[ c(theta), -s(theta), 0, 0],
                       [ s(theta),  c(theta), 0, 0],
                       [ 0,0,1,0],
                       [ 0,0,0,1]])
        
        T = T3*T2*T1

        origin = np.array([[0],
                           [0],
                           [0],
                           [1]])

        pos = np.asarray(np.dot(T, origin))

        T = np.asarray(T)

        # https://eecs.qmul.ac.uk/~gslabaugh/publications/euler.pdf
        if abs(T[2][0]) != 1:
            theta_1 = -math.asin(T[2][0])
            theta_2 = math.pi - theta_1

            psi_1 = math.atan2(T[2][1]/c(theta_1), T[2][2]/c(theta_1))
            psi_2 = math.atan2(T[2][1]/c(theta_2), T[2][2]/c(theta_2))

            phi_1 = math.atan2(T[1][0]/c(theta_1), T[0][0]/c(theta_1))
            phi_2 = math.atan2(T[1][0]/c(theta_2), T[0][0]/c(theta_2))

            if sum(np.abs([theta_1, psi_1, phi_1])) > sum(np.abs([theta_2, psi_2, phi_2])):
                theta_1, psi_1, phi_1 = theta_2, psi_2, phi_2
        else:
            phi_1 = 0
            if T[2][0] == -1:
                theta_1 = math.pi/2
                psi_1 = phi_1 + math.atan2(T[0][1], T[0][2])
            else:
                theta_1 = -math.pi/2
                psi_1 = -phi_1 + math.atan2(T[0][1], T[0][2])

        a = [[pos[0][0]], 
             [pos[1][0]],
             [pos[2][0]], 
             [phi_1],
             [theta_1], 
             [psi_1]]
        return a
    
    def inverse(self, curve_r, phi, theta):
        curve_direction = [c(theta), s(theta), 0]
        l = []
        for i in range(3):
            actuator_point = [self.ACTUATOR_LAYOUT_RADIUS*c(math.radians(120*i)), self.ACTUATOR_LAYOUT_RADIUS*s(math.radians(120*i)), 0]
            new_curve_radius = curve_r - np.dot(actuator_point, curve_direction)
            length = new_curve_radius * phi
            l.append(length)
        return l
    

if __name__ == "__main__":

    

    ACTUATOR_LAYOUT_RADIUS = 116
    ACTUATOR_LENGTH = 150
    ACTUATOR_RADIUS = 20
    BASE_RADIUS = PAYLOAD_RADIUS = ACTUATOR_LAYOUT_RADIUS + ACTUATOR_RADIUS


    IMU_OFFSET = 0
    IMU_NOISE_STD_DEV = 0.1 #deg

    POT_OFFSET = [0,0,0,0,0,0]
    POT_NOISE_STD_DEV = [1,1,1,1,1,1] #mm

    actuator_kin = actuator_kinematic_solver(ACTUATOR_LAYOUT_RADIUS, ACTUATOR_LENGTH)

    # Define parameters
    r_b = 114.91  # Radius of base
    phi_b = 25.73*2  # Angle between base joints
    r_p = 118.32  # Radius of platform
    phi_p = -22.17*2  # Angle between platform joints

    # Create Stewart Platform instance
    sensor_kin = StewartPlatform(r_b, phi_b, r_p, phi_p)

    # REPLACE THIS LINE:
    # actuator_path = [[i, 0, 0] for i in np.linspace(-70, 70, 3000)]
    
    # WITH THIS NEW CODE:
    # Best approach: Mixed systematic and random sampling
    actuator_path = []

    # First, create a systematic grid to ensure coverage of the entire space
    # Using fewer points per dimension but covering the full range
    for l1 in np.linspace(-70, 70, 10):
        for l2 in np.linspace(-70, 70, 10):
            for l3 in np.linspace(-70, 70, 10):
                actuator_path.append([l1, l2, l3])

    # Trim if we've generated too many points
    if len(actuator_path) > 3000:
        # Randomly select 2000 points from the grid
        indices = np.random.choice(len(actuator_path), size=2000, replace=False)
        actuator_path = [actuator_path[i] for i in indices]

    # Add random samples to fill up to 3000 total samples
    # This helps fill in gaps between the grid points
    remaining_samples = 3000 - len(actuator_path)
    for _ in range(remaining_samples):
        l1 = np.random.uniform(-70, 70)
        l2 = np.random.uniform(-70, 70)
        l3 = np.random.uniform(-70, 70)
        actuator_path.append([l1, l2, l3])

    # Add validity checking
    valid_actuator_path = []
    count = 0
    while len(valid_actuator_path) < 3000 and count < len(actuator_path):
        actuator_length = actuator_path[count]
        try:
            actual_pose, _, _, _ = actuator_kin.forward(np.array(actuator_length))
            # Check if the pose is within reasonable bounds
            actual_pose_list = [i[0] for i in actual_pose]
            if all(-500 < actual_pose_list[i] < 500 for i in range(3)) and \
               all(-90 < math.degrees(actual_pose_list[i]) < 90 for i in range(3, 6)):
                valid_actuator_path.append(actuator_length)
        except Exception as e:
            print(f"Skipping invalid configuration {actuator_length}: {e}")
        count += 1

    # If we don't have enough valid configurations, generate more random ones
    while len(valid_actuator_path) < 3000:
        l1 = np.random.uniform(-70, 70)
        l2 = np.random.uniform(-70, 70)
        l3 = np.random.uniform(-70, 70)
        actuator_length = [l1, l2, l3]
        try:
            actual_pose, _, _, _ = actuator_kin.forward(np.array(actuator_length))
            # Check if the pose is within reasonable bounds
            actual_pose_list = [i[0] for i in actual_pose]
            if all(-500 < actual_pose_list[i] < 500 for i in range(3)) and \
               all(-90 < math.degrees(actual_pose_list[i]) < 90 for i in range(3, 6)):
                valid_actuator_path.append(actuator_length)
        except Exception as e:
            continue

    actuator_path = valid_actuator_path

   
    with open(r"C:\Users\rosla\OneDrive\Desktop\ML Model\simulated_datax.csv", "w") as file:
        file.write(','.join(["l1", "l2", "l3", "Actual pose x", "Actual pose y", "Actual pose z", "Actual pose roll", "Actual pose pitch", "Actual pose yaw", "IMU roll", "IMU pitch", "IMU yaw", "Pot 1 distance", "Pot 2 distance", "Pot 3 distance", "Pot 4 distance", "Pot 5 distance", "Pot 6 distance", "Estimated pose x", "Estimated pose y", "Estimated pose z", "Estimated pose roll", "Estimated pose pitch", "Estimated pose yaw",])+"\n")
        count = 0
        for actuator_length in actuator_path:
            actual_pose, _, _, _ = actuator_kin.forward(np.array(actuator_length)) #Calculate platform position from actuator lengths
            actual_pose = [i[0] for i in actual_pose]
            for i in range(3,6): actual_pose[i] = math.degrees(actual_pose[i])
            #print(actual_pose)

            sensor_lengths =  sensor_kin.getIK(actual_pose) #Calculate sensor lengths from actual pose
            sensor_lengths = np.linalg.norm(sensor_lengths, axis=1)
            #print(sensor_lengths)
            #sensor_kin.plot()

            imu_readings = [i + np.random.normal(IMU_OFFSET, IMU_NOISE_STD_DEV, 1)[0] for i in actual_pose[3:]] #simulate sensor readings
            #print(imu_readings)
            pot_readings = [sensor_lengths[i] + np.random.normal(POT_OFFSET[i], POT_NOISE_STD_DEV[i], 1)[0] for i in range(6)]
            #print(pot_readings)

            #inverse kinematics
            starting_pose = [0, 0, 250, 0, 0, 0]  # Initial guess for the pose
            plot=False
            estimated_pose = sensor_kin.getFK(starting_pose, pot_readings, plot)

            #save to file
            data = [*actuator_length, *actual_pose,*imu_readings, *pot_readings, *estimated_pose]
            data = [str(i) for i in data]
            file.write(','.join(data)+"\n")
            print(count)
            count += 1


