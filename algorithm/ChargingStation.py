class ChargingStation:
    def __init__(self, id, lane, x, y):
        self.id = id
        self.Lane = lane
        self.X = x
        self.Y = y
        self.VehiclesCharging = 0
        self.Price = 0
        self.Power = 0
        self.Effciency = 0
