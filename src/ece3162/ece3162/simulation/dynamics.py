"""Deterministic bounded double-integrator simulation with estimate delay/noise."""

from __future__ import annotations
from dataclasses import dataclass
from collections import deque
import numpy as np


@dataclass(frozen=True, slots=True)
class DynamicsConfig:
    dt: float = 0.02
    max_velocity: float = 2.0
    max_acceleration: float = 2.0
    position_noise_std: float = 0.0
    velocity_noise_std: float = 0.0
    feedback_delay: float = 0.0
    seed: int = 0


@dataclass(slots=True)
class TranslationalState:
    position: np.ndarray
    velocity: np.ndarray
    timestamp: float
    valid: bool = True


class DoubleIntegratorSimulator:
    def __init__(self, position: np.ndarray, velocity: np.ndarray | None=None,
                 config: DynamicsConfig = DynamicsConfig()) -> None:
        if config.dt <= 0 or config.max_velocity <= 0 or config.max_acceleration <= 0: raise ValueError("limits must be positive")
        self.config=config; self.position=np.asarray(position,float).copy()
        self.velocity=np.zeros(3) if velocity is None else np.asarray(velocity,float).copy()
        self.time=0.0; self.rng=np.random.default_rng(config.seed); self.history=deque()
        self.history.append(TranslationalState(self.position.copy(), self.velocity.copy(), 0.0))

    def step(self, acceleration: np.ndarray) -> TranslationalState:
        a=np.asarray(acceleration,float); n=np.linalg.norm(a)
        if n > self.config.max_acceleration: a=a*self.config.max_acceleration/n
        dt=self.config.dt; self.position += self.velocity*dt + .5*a*dt*dt; self.velocity += a*dt
        speed=np.linalg.norm(self.velocity)
        if speed > self.config.max_velocity: self.velocity *= self.config.max_velocity/speed
        self.time += dt; state=TranslationalState(self.position.copy(),self.velocity.copy(),self.time)
        self.history.append(state)
        while len(self.history)>2 and self.history[1].timestamp <= self.time-self.config.feedback_delay: self.history.popleft()
        return state

    def estimated_state(self) -> TranslationalState:
        state=self.history[0]
        return TranslationalState(state.position+self.rng.normal(0,self.config.position_noise_std,3),
            state.velocity+self.rng.normal(0,self.config.velocity_noise_std,3),state.timestamp,state.valid)
