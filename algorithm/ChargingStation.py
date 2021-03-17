class ChargingStation:
    def __init__(self, id, lane, x, y, startPos, power, efficiency):
        self.id = id
        self.Lane = lane.split('_')[0]
        self.StartPos = startPos
        self.X = x
        self.Y = y
        self.VehiclesCharging = 0
        self.Price = 0
        self.Power = power
        self.Efficiency = efficiency
        self.Duration = 0
        self.DistanceFromStart = 0
        self.DistanceFromDivider = 0
