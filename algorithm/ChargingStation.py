class ChargingStation:
    def __init__(self, id, lane, x, y, startPos, endPos, power, efficiency):
        self.id = id
        self.Lane = lane.split('_')[0]
        self.StartPos = startPos
        self.EndPos = endPos
        self.X = x
        self.Y = y
        self.VehiclesCharging = 0
        self.Price = 0
        self.Power = power
        self.Efficiency = efficiency
        self.ChargePerStep = (power * efficiency) / 3600
        self.Duration = 0
        self.DistanceFromStart = 0
        self.DistanceFromDivider = 0
        self.Score = 0

    def __str__(self):
        return '[' + (str(self.id) + ", " + str(self.Lane)
                + ", " + str(self.StartPos) + ", " + str(self.EndPos)
                + ", " + str(self.VehiclesCharging) + ", " + str(self.Price)
                + ", " + str(self.Power) + ", " + str(self.Efficiency)
                + ", " + str(self.Duration) + ", " + str(self.DistanceFromStart)
                + ", " + str(self.DistanceFromDivider)) + ']'
