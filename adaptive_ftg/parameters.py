class Parameters:
    def __init__(self):
        # Static Params
        self.BEST_POINT_CONV_SIZE = 80  # Reduced: Don't smooth the gap too much on small tracks
        self.EDGE_GUARD_DEG = 12.0      
        self.TTC_HARD_BRAKE = 0.55      
        self.TTC_SOFT_BRAKE = 0.9       
        self.FWD_WEDGE_DEG = 8.0        
        self.STEER_RATE_LIMIT = 0.3     # Increased: Real servos need permission to move faster
        self.CENTER_BIAS_ALPHA = 0.5

        # Dynamic Parameters (Defaults)
        self.BUBBLE_RADIUS = 40 
        self.MAX_LIDAR_DIST = 3.0 
        self.CURRENT_SPEED = 2.0 
        self.MAX_STEER_ABS = 0.7 
        self.STEER_SMOOTH_ALPHA = 0.5 
        self.PREPROCESS_CONV_SIZE = 3 

        self.state = "CORNER"
        self.change_num = 0

    def __updateParameters(self, input:list):
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
        if inputState not in ["STRAIGHT", "CORNER", "HAIRPIN"]:
            return

        changed = False
        if self.state != inputState:
            self.state = inputState
            self.change_num += 1
            changed = True

        # Formula: s = (1.0 - ALPHA) * new + ALPHA * old
        # High Alpha (0.9) = Mostly Old (Smooth/Slow)
        # Low Alpha (0.1) = Mostly New (Fast/Twitchy)

        if inputState == "STRAIGHT":
            if changed: print(f"[STATE {self.change_num}] ➡️ STRAIGHT")
            
            # Speed: 3.5 (Max for small room)
            # Alpha: 0.85 (Very Smooth - prevents wobbling on straights)
            # Steer: 0.2 (Clamp tight - don't let it weave)
            params = [30, 4.0, 3.5, 0.2, 0.85, 5]
            self.__updateParameters(params)

        elif inputState == "CORNER":
            if changed: print(f"[STATE {self.change_num}] ⤴️ CORNER")
            
            # Speed: 2.0 (Safe pace)
            # Alpha: 0.5 (Balanced)
            params = [45, 2.5, 2.0, 0.5, 0.5, 3]
            self.__updateParameters(params)

        elif inputState == "HAIRPIN":
            if changed: print(f"[STATE {self.change_num}] ↩️ HAIRPIN")
            
            # Speed: 1.0 (Crawl speed to ensure grip)
            # Alpha: 0.1 (Responsive - Turn NOW)
            # Bubble: 40 (Small enough to fit, big enough for safety)
            params = [40, 1.5, 1.0, 0.8, 0.1, 1]   
            self.__updateParameters(params)