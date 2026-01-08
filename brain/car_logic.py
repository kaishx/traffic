# kaile to keenan: this code is for the car logic
# inputs should be raw data based on hall sensors or wtv
# the logic should ideally make calculations for position and velocity
# then it should give an output to car.py of how the car should move to the next node
# then it should poll traffic_logic for its next move

class CarBrain:
    """
    The "driver" for a car. It takes in sensor data and makes decisions.
    This class separates the decision-making logic from the car's physical representation.
    """
    def __init__(self, car):
        """
        Initializes the Brain.
        :param car: The car object this brain will control.
        """
        self.car = car

    def decide(self, sensor_data, traffic_rules):
        is_light_red = traffic_rules.get('is_light_red', False)
        is_car_too_close = sensor_data.get('is_car_ahead', False)

        if is_light_red or is_car_too_close:
            self.car.brake()
        else:
            self.car.accelerate()