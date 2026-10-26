"""Modern, deterministic PID primitive replacing the ROS-coupled legacy class."""
from __future__ import annotations
from dataclasses import dataclass
import math

@dataclass(frozen=True,slots=True)
class PIDConfig:
    kp:float;ki:float=0.;kd:float=0.;minimum:float=-math.inf;maximum:float=math.inf;integral_limit:float=math.inf

class PID:
    def __init__(self,config:PIDConfig)->None:self.config=config;self.reset()
    def reset(self)->None:self.integral=0.;self.previous_error:float|None=None
    def update(self,error:float,dt:float)->float:
        if not math.isfinite(error) or dt<=0 or not math.isfinite(dt):raise ValueError("finite error and positive dt required")
        self.integral=max(-self.config.integral_limit,min(self.config.integral_limit,self.integral+error*dt))
        derivative=0. if self.previous_error is None else (error-self.previous_error)/dt;self.previous_error=error
        value=self.config.kp*error+self.config.ki*self.integral+self.config.kd*derivative
        return max(self.config.minimum,min(self.config.maximum,value))
