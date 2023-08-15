class Ultrasonic:
    def __init__(self):
        self.l = 140
        self.r = 140
        self.br = 140
        self.b = 140
        self.bl = 140

    def ultra_data(self, data):
        self.l = data[2]
        self.r = data[4]
        self.br = data[5]
        self.b = data[6]
        self.bl = data[7]

    def ultra_steer(self):
        global ultra
        # 멈추는 코드
        if self.b < 10 and self.bl < 25:
            return True
        return False