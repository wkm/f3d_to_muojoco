# Velocity Actuator Bug Fix

## Problem Identified

Velocity actuators were not working because the `ctrlrange` was set to `-1 1`, which means:
- The desired velocity is limited to ±1 rad/s
- This is extremely slow: ~9.5 degrees/second or 0.16 rotations/second
- Essentially imperceptible motion!

## Root Cause

**Velocity actuators work differently than position actuators:**
- **Position actuators**: Control signal is normalized (-1 to 1 is fine)
- **Velocity actuators**: Control signal represents **actual desired velocity** in rad/s or units/s
- Our code incorrectly used `-1 1` for all actuator types

## The Fix

### Changed Default Parameters

**Before:**
```python
# Velocity control defaults
ET.SubElement(default_elem, "velocity", {"kv": "10", "ctrlrange": "-1 1"})
```

**After:**
```python
# Velocity control defaults (control signal is desired velocity in rad/s or units/s)
# Using reasonable velocity range for wheels/joints: ±20 rad/s (~3.2 rev/s)
ET.SubElement(default_elem, "velocity", {"kv": "100", "ctrlrange": "-20 20"})
```

### Changes Made

1. **`ctrlrange`: `-1 1` → `-20 20`**
   - Now allows velocities up to ±20 rad/s (~3.2 rotations/second)
   - Suitable for wheels, joints, and most robotic applications
   - Based on MuJoCo examples showing 2-3 rad/s for manipulators, higher for wheels

2. **`kv`: `10` → `100`**
   - Increased damping gain for better control authority
   - Higher gain = stronger velocity tracking

### Updated Actuator Generation Logic

Also updated `_add_actuators()` to use appropriate control ranges based on actuator type:

```python
if self.actuator_type == "position":
    # Use joint limits if available, else normalized range
    attrs["ctrlrange"] = limits if (has_limits and limits) else "-1 1"
elif self.actuator_type == "velocity":
    # Always use velocity range (rad/s or units/s)
    attrs["ctrlrange"] = "-20 20"  # ±20 rad/s for all velocity actuators
else:  # motor
    # Normalized range for torque control
    attrs["ctrlrange"] = "-1 1"
```

## Expected Output (Fixed)

For Simple2Wheel with velocity actuators:

```xml
<default>
  <velocity kv="100" ctrlrange="-20 20" />
</default>

<actuator>
  <velocity name="Revolute_1_vel" joint="Revolute_1" ctrlrange="-20 20" />
  <velocity name="Revolute_2_vel" joint="Revolute_2" ctrlrange="-20 20" />
</actuator>
```

## Testing the Fix

1. **Re-export** Simple2Wheel with velocity actuators selected
2. **Load in MuJoCo**: `simulate Simple2Wheel.xml`
3. **Move the sliders** in the control panel
4. **Wheels should now spin** at visible speeds!

## Velocity Range Explanation

| ctrlrange | Meaning | Speed |
|-----------|---------|-------|
| `-1 1` (OLD) | ±1 rad/s | ~0.16 rev/s (too slow!) |
| `-20 20` (NEW) | ±20 rad/s | ~3.2 rev/s (visible motion) |

At full control (+20 or -20):
- **20 rad/s = ~190 RPM = ~3.2 rotations/second**
- Much more appropriate for wheels and robotic joints

## References

- [MuJoCo Documentation - Modeling](https://mujoco.readthedocs.io/en/stable/modeling.html)
- Typical velocity limits for robot manipulators: 2-3 rad/s
- Typical velocity limits for wheels: 10-50 rad/s
- MuJoCo velocity actuators use control signal as desired velocity (not normalized)

## Files Changed

- `F3DToMuJoCo/F3DToMuJoCo.py`:
  - `_add_default_section()` - Updated velocity defaults
  - `_add_actuators()` - Added type-specific control range logic
