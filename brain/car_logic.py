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
        """
        The core logic loop for the car's brain.
        
        :param sensor_data: Information about the car's immediate surroundings
                            (e.g., distance to car in front).
        :param traffic_rules: Information from the higher-level traffic logic
                              (e.g., is the traffic light red?).
        """
        # Example decision:
        is_light_red = traffic_rules.get('is_light_red', False)
        is_car_too_close = sensor_data.get('is_car_ahead', False)

        if is_light_red or is_car_too_close:
            self.car.brake()
        else:
            self.car.accelerate()