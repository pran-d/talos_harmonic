
class PDController():
    
    def __init__(self, Kp=50, Kd=20):
        self.Kp = Kp
        self.Kd = Kd

    def doControl(self, qMeas, vMeas, qDes, vDes):
        output = self.Kp * (qDes - qMeas) + self.Kd * (vDes - vMeas)
        return output