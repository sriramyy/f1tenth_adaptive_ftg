import numpy as np

class Logic:
    def __init__(self):
        # TUNING THRESHOLDS
        # determines WHEN the car changes states

        # average dist > 8 means on straight
        self.THRESH_STRAIGHT = 8.0 
        # average dist < 3.5 means hairpin/slow section
        self.THRESH_HAIRPIN = 3.5


    def getFrontDist(self, ranges) -> float:
        """Calculates the average distance from front"""
        mid_idx = len(ranges) // 2
        window_size = 40 

        # slice the center array
        center_chunk = ranges[mid_idx - window_size : mid_idx + window_size]

        # filter out invalid (0, inf) points
        valid_points = center_chunk[np.isfinite(center_chunk)]

        if len(valid_points) == 0:
            return 0.0
        
        return np.mean(valid_points)


    def detState(self, ranges) -> str:
        """Determines the state based on lidar data"""

        front_dist = self.getFrontDist(ranges)

        if front_dist > self.THRESH_STRAIGHT:
            return "STRAIGHT"
        elif front_dist < self.THRESH_HAIRPIN:
            return "HAIRPIN"
        else:
            return "CORNER"