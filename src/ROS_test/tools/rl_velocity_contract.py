"""Pure-Python safety contract for the chassis-level RL residual."""
from __future__ import annotations

from dataclasses import dataclass
import math


def _triple(values, name):
    result=tuple(float(value) for value in values)
    if len(result)!=3 or not all(math.isfinite(value) for value in result):
        raise ValueError(f"{name} must contain three finite values")
    return result


def _clip(value, limit):
    return min(max(value,-limit),limit)


def _limit_velocity(command, limits):
    vx,vy,wz=command
    planar=math.hypot(vx/limits.velocity[0],vy/limits.velocity[1])
    scale=1/max(1.,planar)
    return vx*scale,vy*scale,_clip(wz,limits.velocity[2])


@dataclass(frozen=True)
class Limits:
    velocity: tuple[float,float,float]
    acceleration: tuple[float,float,float]
    residual_fraction: float = .05

    def __post_init__(self):
        velocity=_triple(self.velocity,"velocity")
        acceleration=_triple(self.acceleration,"acceleration")
        if any(value<=0 for value in (*velocity,*acceleration)):
            raise ValueError("velocity and acceleration limits must be positive")
        if not math.isfinite(self.residual_fraction) or not 0 < self.residual_fraction <= .3:
            raise ValueError("residual_fraction must be in (0, .3]")
        object.__setattr__(self,"velocity",velocity)
        object.__setattr__(self,"acceleration",acceleration)


def mix(classic, action, limits, previous=(0,0,0), dt=.02, *, classic_fresh=True,
        residual_fresh=True, health_ok=True, estop=False):
    """Return ``(command, mode, saturated)`` without exposing motor control."""
    if estop:return (0.,0.,0.),"estop",False
    if not health_ok:return (0.,0.,0.),"health_stop",False
    if not classic_fresh:return (0.,0.,0.),"classic_timeout",False
    classic=_triple(classic,"classic")
    previous=_triple(previous,"previous")
    if residual_fresh:
        action=_triple(action,"action")
        if any(abs(value)>1 for value in action):
            raise ValueError("normalized action must stay in [-1, 1]")
        residual=tuple(value*limits.residual_fraction*scale
                       for value,scale in zip(action,limits.velocity))
        mode="classic_plus_residual"
    else:
        residual=(0.,0.,0.)
        mode="classic_only_rl_timeout"
    requested=tuple(base+delta for base,delta in zip(classic,residual))
    limited=_limit_velocity(requested,limits)
    if not math.isfinite(dt) or dt<=0:raise ValueError("dt must be positive and finite")
    output=tuple(old+_clip(new-old,acceleration*dt)
                 for old,new,acceleration in zip(previous,limited,limits.acceleration))
    # Axis-wise slew can leave the planar ellipse while changing direction;
    # reapply the public command envelope just like chassis_control_node.cpp.
    output=_limit_velocity(output,limits)
    saturated=any(abs(a-b)>1e-12 for a,b in zip(requested,output))
    return output,mode,saturated
