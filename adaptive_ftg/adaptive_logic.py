import numpy as np

class Logic:
    def __init__(self):
        # --- REAL WORLD / SMALL TRACK THRESHOLDS ---
        
        # Reduced from 8.0 -> 3.0
        # On a small track, if you see 3m of space, that IS a straight.
        self.THRESH_STRAIGHT = 3.0 
        
        # Reduced from 3.5 -> 1.5
        # Only panic if the wall is literally right in front of us.
        self.THRESH_HAIRPIN = 1.5

    def getFrontDist(self, ranges) -> float:
        mid_idx = len(ranges) // 2
        window_size = 40 
        center_chunk = ranges[mid_idx - window_size : mid_idx + window_size]
        valid_points = center_chunk[np.isfinite(center_chunk)]

        if len(valid_points) == 0:
            return 0.0
        
        return np.mean(valid_points)

    def detState(self, ranges) -> str:
        front_dist = self.getFrontDist(ranges)

        if front_dist > self.THRESH_STRAIGHT:
            return "STRAIGHT"
        elif front_dist < self.THRESH_HAIRPIN:
            return "HAIRPIN"
        else:
            return "CORNER"