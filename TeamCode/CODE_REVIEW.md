# Code Review - Chassis Control Fixes

## Commit Verified: Fix chassis control issues (#20)

### Summary of Changes
The following changes were verified to address chassis control issues:

1. **SuperChassis.java**
   - `driveHolonomic` updated with correct field-centric math.
   - Strafe input inverted (`strafe = -strafe`).
   - Motor initialization now sets directions based on `ChassisConstants`.
   - `ZeroPowerBehavior` set to `FLOAT`.

2. **SimpleMecanumChassis.java**
   - Motor directions explicitly configured:
     - FL/FR: REVERSE
     - BL/BR: FORWARD
   - `ZeroPowerBehavior` set to `FLOAT`.
   - Field-centric logic updated with inverted strafe.

3. **TeleopMode.java**
   - Joystick inputs for strafe (`left_stick_x`) and turn (`right_stick_x`) are negated to ensure correct robot direction relative to driver input.
