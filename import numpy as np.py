import numpy as np

class AdvancedController:
    def __init__(self):
        # PID gains
        self.kp = 0.5  # Proportional gain
        self.ki = 0.1 # Integral gain
        self.kd = 0.05  # Derivative gain
        
        # Feed-forward gain
        self.kf = 0.2  # Feed-forward gain
        
        # Anti-windup settings
        self.windup_limit = 0.9
        
        # Internal state
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_target = 0.0
        self.dt = 0.01  # Assuming 100Hz control frequency
        
        # Rate limiters
        self.max_steer_rate = 0.05  # Maximum steering rate change
        self.prev_steer = 0.0
        
        # Smoothing filter
        self.alpha = 0.2  # Smoothing factor
        self.filtered_target = 0.0
    
    def reset(self):
        """Reset controller state."""
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_target = 0.0
        self.prev_steer = 0.0
        self.filtered_target = 0.0
    
    def update(self, current_lataccel, road_lataccel, v_ego, desired_lataccel):
        """
        Main control function to determine steering based on current state.
        
        Args:
            current_lataccel: Current lateral acceleration of the car
            road_lataccel: Lateral acceleration due to road roll
            v_ego: Car velocity
            desired_lataccel: Target lateral acceleration
            
        Returns:
            steer_action: Steering action to apply
        """
        # Apply smoothing to desired lataccel to reduce jerk
        self.filtered_target = self.alpha * desired_lataccel + (1 - self.alpha) * self.filtered_target
        
        # Calculate error (accounting for road roll)
        error = self.filtered_target - current_lataccel
        
        # Calculate derivative term (use band-limited derivative)
        derivative = (error - self.prev_error) / self.dt
        
        # Update integral term with anti-windup
        self.integral += error * self.dt
        self.integral = np.clip(self.integral, -self.windup_limit, self.windup_limit)
        
        # Calculate feed-forward term (helps with responsiveness)
        # The rate of change of desired acceleration can predict future steering needs
        ff_term = (self.filtered_target - self.prev_target) / self.dt * self.kf
        
        # Adjust gains based on velocity (gain scheduling)
        # At higher speeds, we need less aggressive steering
        vel_factor = 1.0 / max(1.0, v_ego / 10.0)
        
        # Calculate PID + FF output
        steer_raw = (
            self.kp * error * vel_factor + 
            self.ki * self.integral * vel_factor + 
            self.kd * derivative * vel_factor +
            ff_term
        )
        
        # Apply rate limiting to reduce jerk
        steer_rate = (steer_raw - self.prev_steer) / self.dt
        limited_steer_rate = np.clip(steer_rate, -self.max_steer_rate, self.max_steer_rate)
        steer_action = self.prev_steer + limited_steer_rate * self.dt
        
        # Update state for next iteration
        self.prev_error = error
        self.prev_target = self.filtered_target
        self.prev_steer = steer_action
        
        return steer_action

# Register the controller in the format expected by the simulation framework
def get_controller():
    return AdvancedController()
