class Parameters:
    def __init__(self):
        # Static Params (do not change)
        self.BEST_POINT_CONV_SIZE = 120 
        self.EDGE_GUARD_DEG = 12.0      
        self.TTC_HARD_BRAKE = 0.55      
        self.TTC_SOFT_BRAKE = 0.9       
        self.FWD_WEDGE_DEG = 8.0        
        self.STEER_RATE_LIMIT = 0.14
        self.CENTER_BIAS_ALPHA = 0.5

        # Dynamic Parameters
        self.BUBBLE_RADIUS = 70        # 
        self.MAX_LIDAR_DIST = 6.0      #
        self.CURRENT_SPEED = 4.0       #
        self.MAX_STEER_ABS = 0.7       # Angle Limit
        self.STEER_SMOOTH_ALPHA = 0.5  # 0.1 = Slow/Smooth, 1.0 = Instant
        self.PREPROCESS_CONV_SIZE = 3  # determines the "sharpness" of vision 

        # State
        self.state = "CORNER"


    def __updateParameters(self, input:list):
        """Internal function that converts a list into the updated dynamic parameters"""
        if len(input) != 6:
            print("[ERROR]: __updateParameters called with incorrect number of arguments.")
            return

        self.BUBBLE_RADIUS = input[0]
        self.MAX_LIDAR_DIST = input[1]
        self.CURRENT_SPEED = input[2]
        self.MAX_STEER_ABS = input[3]
        self.STEER_SMOOTH_ALPHA = input[4]
        self.PREPROCESS_CONV_SIZE = input[5]


    def setState(self, inputState:str):
        """Sets the state and updates the dynamic parameters.
        Valid States include: STRAIGHT, CORNER, HAIRPIN"""

        if inputState not in ["STRAIGHT", "CORNER", "HAIRPIN"]:
            print(f"[ERROR] setState called with an invalid state")
            return

        changed = False

        if self.state != inputState:
            self.state = inputState
            changed = True

        # Format: [Bubble, Lookahead, Speed, SteerAbs, Alpha, Conv]
        if inputState == "STRAIGHT":
            if changed: print(f"[STATE CHANGE] ➡️ STRAIGHT")
            # when straight reduce bubble, increase lookahead and speed
            # don't let car turn too hard, make steering more smooth, reduce the sharpness of vision
            params = [40, 10.0, 8.0, 0.15, 0.1, 5]
            self.__updateParameters(params)

        elif inputState == "CORNER":
            if changed: print(f"[STATE CHANGE] ⤴️ CORNER")
            # when corner
            params = [70, 6.0, 4.5, 0.5, 0.5, 3]
            self.__updateParameters(params)

        elif inputState == "HAIRPIN":
            if changed: print(f"[STATE CHANGE] ↩️ HAIRPIN")
            params = [120, 3.0, 1.5, 0.8, 1.0, 1]
            self.__updateParameters(params)