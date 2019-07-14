class Admittance_control(object):
    def __int__(self):
        self.speed_1 = 0

    def df(self, speed):
        m_d = 3
        gain = 1
        h = 0.001
        f = h * (-self.damping * speed + gain * self.external_tau) / m_d
        return f

    def fourth_order_RKT(self):
        k1 = self.df(self.speed_1)
        k2 = self.df(self.speed_1 + k1 / 2)
        k3 = self.df(self.speed_1 + k2 / 2)
        k4 = self.df(self.speed_1 + k3)
        self.speed_2 = self.speed_1 + (k1 + 2 * k2 + 2 * k3 + k4) / 6
        self.speed_1 = self.speed_2

    def admittance_controller(self, external_tau, damping):
        self.external_tau = external_tau
        self.damping = damping
        self.fourth_order_RKT()
        return self.speed_2