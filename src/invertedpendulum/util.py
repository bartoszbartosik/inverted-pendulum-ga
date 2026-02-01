
def get_value_for_interval(t, intervals):
    for time_interval, position in intervals.items():
        if time_interval[0] <= t <= time_interval[1]:
            return position
    return ValueError("Time value out of the defined intervals.")