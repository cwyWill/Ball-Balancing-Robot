import numpy as np
import quaternion

class AntiWindupPID:
    def __init__(self, Ts, Kp, Ki=0, Kd=0, windup_limits=(None, None), output_limits=(None, None)):
        self.Ts = Ts
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.windup_limits = windup_limits
        self.output_limits = output_limits
        self.prev_error = 0
        self.integral = 0

    def update(self, reference, measurement):
        error = reference - measurement
        # integral term with anti-windup
        self.integral += self.Ki * error * self.Ts
        if self.windup_limits[0] is not None and self.integral < self.windup_limits[0]:
            self.integral = self.windup_limits[0]
        if self.windup_limits[1] is not None and self.integral > self.windup_limits[1]:
            self.integral = self.windup_limits[1]

        # derivative term
        derivative = self.Kd * (error - self.prev_error) / self.Ts if self.Ts > 0 else 1
        self.prev_error = error

        output = self.Kp * error + self.integral + derivative

        if self.output_limits[0] is not None and output < self.output_limits[0]:
            output = self.output_limits[0]
        if self.output_limits[1] is not None and output > self.output_limits[1]:
            output = self.output_limits[1]
        return output

class BallController:
    def __init__(self, rate=1/56.03):
        # controller output unit: degree
        self.pid_x = AntiWindupPID(rate, Kp=.5, Ki=0.00, Kd=0.00, windup_limits=(-1, 1), output_limits=(-10, 10))
        self.pid_y = AntiWindupPID(rate, Kp=.5, Ki=0.00, Kd=0.00, windup_limits=(-1, 1), output_limits=(-10, 10))
        self.Ts = rate

    def update(self, reference, measurement):
        # print("Reference: ", reference, "Measurement: ", measurement)
        control_x = self.pid_x.update(reference[0], measurement[0])
        control_y = self.pid_y.update(reference[1], measurement[1])
        print("Control output: ", control_x, control_y)
        return np.array([control_x, control_y])


class BallBalance:
    def __init__(self):
        self.l1 = 60
        self.l2 = 80
        self.l3 = 110
        self.l4 = 60
        self.g  = 9.81
        self.delta_list = [0, 2*np.pi/3, 4*np.pi/3]
    
    def IK(self, axis, angle, h):
        axis = axis / np.linalg.norm(axis)
        axis *= np.sin(angle/2)
        qop = np.quaternion(np.cos(angle/2), axis[0], axis[1], axis[2])
        theta_list = [None] * 3
        for i in range(3):
            delta = self.delta_list[i]
            p = np.quaternion(0, self.l3*np.cos(delta), self.l3*np.sin(delta), 0)
            rp = qop * p * qop.conj()
            qr = rp.w
            qx = rp.x
            qy = rp.y
            qz = rp.z

            d = np.sqrt(qx**2 + qy**2)
            r = np.sqrt( (h+qz)**2 + ( d - self.l4)**2 )
            phi = np.arctan2( h+qz, (d-self.l4) )
            cpsi = ( r**2 + self.l1**2 - self.l2**2 ) / ( 2 * r * self.l1 )
            psi = np.arctan2( np.sqrt(1-cpsi**2), cpsi )
            theta_list[i] = phi - psi
        return theta_list

    # def IK(self, alpha, beta, h):
    #     ''' Inverse Kinematics: calculate the servo angles from platform orientation and height
    #     Arguments:
    #         alpha: rotation around x axis (rad)
    #         beta: rotation around y axis (rad)
    #         h: height of the platform (mm)
    #     Returns:
    #         theta_list: list of servo angles (rad)
    #     '''

    #     theta_list = [None] * 3
    #     for i in range(3):
    #         delta = self.delta_list[i]
    #         d = np.cos(alpha) * np.cos(beta) / np.sqrt(np.cos(alpha)**2 * np.sin(beta)**2 * np.cos(delta)**2 + 
    #             np.sin(alpha)**2 * np.sin(delta)**2 + np.cos(alpha)**2 * np.cos(beta)**2 \
    #             - 2 * np.cos(alpha) * np.sin(alpha) * np.sin(beta) * np.cos(delta) * np.sin(delta)) * self.l3
    #         px = d * np.cos(delta)
    #         py = d * np.sin(delta)
    #         pz = np.sqrt(self.l3**2 - d**2) + h
    #         P3 = np.array([px, py, pz])
    #         P1 = np.array([self.l4 * np.cos(delta), self.l4 * np.sin(delta), 0])
    #         r_sq = np.sum(np.square(P3 - P1))
    #         # r_sq = vector_square(P3, P1)
    #         r = np.sqrt(r_sq)
    #         cphi = (r**2 + self.l1**2 - self.l2**2) / (2 * r * self.l1)
    #         sphi = np.sqrt(1-cphi**2)
    #         phi = np.arctan2(sphi, cphi)
            
    #         s_sq = d**2 + pz**2
    #         cpsi = (self.l4**2 + r_sq - s_sq) / (2 * self.l4 * r)
    #         spsi = np.sqrt(1-cpsi**2)
    #         psi = np.arctan2(spsi, cpsi)
            
    #         t = np.pi - phi - psi
    #         theta_list[i] = t
    #     return theta_list


