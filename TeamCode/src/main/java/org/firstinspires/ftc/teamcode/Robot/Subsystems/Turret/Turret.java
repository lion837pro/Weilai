package org.firstinspires.ftc.teamcode.Robot.Subsystems.Turret;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.util.ElapsedTime;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.NullCommand;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.MotorEx;

/**
 * Turret Subsystem
 *
 * A rotating turret that aims the shooter at targets.
 * Uses a single motor with 8:1 gear ratio for precise positioning.
 * Supports position control, vision-based auto-alignment, and odometry-based targeting.
 */
public class Turret implements Subsystem {

    public static final Turret INSTANCE = new Turret();

    // Hardware
    private MotorEx motor;

    // State tracking
    private double currentAngle = 0;          // Current angle in degrees
    private double targetAngle = 0;           // Target angle in degrees
    private boolean hasTarget = false;        // Position control active?
    private boolean isAligning = false;       // Vision alignment active?
    private boolean isOdometryTargeting = false; // Odometry-based targeting active?
    private boolean isReturningToCenter = false; // Returning to center after tracking?

    // Control state
    private double targetTicks = 0;           // Target encoder position
    private double currentPower = 0;
    private double encoderOffset = 0;         // Virtual encoder reset offset

    // PID state
    private double lastError = 0.0;
    private long lastPIDTime = 0;

    // Power efficiency: track if motor is idle
    private boolean isIdle = true;

    // Alignment state (for vision-based auto-align)
    private double alignError = 0;            // Error from vision (tx)
    private double lastAlignError = 0;

    // Odometry targeting state
    private double odometryTargetAngle = 0;   // Target angle from odometry calculation
    private double robotHeading = 0;          // Current robot heading from odometry

    // Timing
    private ElapsedTime moveTimer = new ElapsedTime();

    // Default command
    private Command defaultCommand = new NullCommand();

    @Override
    public void initialize() {
        // Initialize motor
        try {
            this.motor = new MotorEx(TurretConstants.TURRET_MOTOR_NAME);
            if (TurretConstants.MOTOR_INVERTED) {
                motor.reversed();
            }
            motor.brakeMode();
            resetEncoder();
        } catch (Exception e) {
            ActiveOpMode.telemetry().addData("Turret Error", "Motor not found: " + e.getMessage());
            this.motor = null;
        }
    }

    @NonNull
    @Override
    public Command getDefaultCommand() {
        return defaultCommand;
    }

    public void setDefaultCommand(Command command) {
        this.defaultCommand = command;
    }

    @Override
    public void periodic() {
        if (motor == null) return;

        // Update current angle from encoder
        currentAngle = TurretConstants.ticksToDegrees(getCurrentTicks());

        // Position control loop (manual go-to-angle)
        if (hasTarget && !isAligning && !isOdometryTargeting) {
            runPositionPID();
        }

        // Vision alignment loop (Limelight-based)
        if (isAligning) {
            runAlignmentPID();
        }

        // Odometry targeting loop (field position-based)
        if (isOdometryTargeting) {
            runOdometryPID();
        }

        // Update telemetry
        if (TurretConstants.ENABLE_TELEMETRY) {
            updateTelemetry();
        }
    }

    // ===== POSITION CONTROL =====

    /**
     * Move turret to a specific angle (degrees from center)
     */
    public void goToAngle(double degrees) {
        if (motor == null) return;

        // Clamp to valid range
        degrees = TurretConstants.clampAngle(degrees);

        targetAngle = degrees;
        targetTicks = TurretConstants.degreesToTicks(degrees);

        // Reset PID state
        lastError = 0.0;
        lastPIDTime = 0;

        // Set position mode, clear other modes
        hasTarget = true;
        isAligning = false;
        isOdometryTargeting = false;
        isReturningToCenter = false;
        moveTimer.reset();
    }

    /**
     * Move turret to center position (0 degrees)
     */
    public void goToCenter() {
        goToAngle(TurretConstants.POSITION_CENTER);
    }

    /**
     * Return to center while unwinding cables.
     * If turret is at positive angle (turned right/clockwise), returns by turning left (counter-clockwise).
     * If turret is at negative angle (turned left/counter-clockwise), returns by turning right (clockwise).
     * This prevents cable twisting from accumulated turns.
     */
    public void returnToCenterUnwinding() {
        if (motor == null) return;

        // Start position control to center
        targetAngle = TurretConstants.POSITION_CENTER;
        targetTicks = TurretConstants.degreesToTicks(TurretConstants.POSITION_CENTER);

        // Reset PID state
        lastError = 0.0;
        lastPIDTime = 0;

        hasTarget = true;
        isAligning = false;
        isOdometryTargeting = false;
        isReturningToCenter = true;
        moveTimer.reset();
    }

    /**
     * Check if turret is currently returning to center
     */
    public boolean isReturningToCenter() {
        return isReturningToCenter;
    }

    /**
     * Adjust turret angle by a delta amount
     */
    public void adjustAngle(double deltaDegrees) {
        goToAngle(targetAngle + deltaDegrees);
    }

    /**
     * Position PID control loop
     */
    private void runPositionPID() {
        long currentTime = System.nanoTime();
        double dt = lastPIDTime == 0 ? 0.02 : (currentTime - lastPIDTime) / 1e9;
        lastPIDTime = currentTime;

        double error = targetTicks - getCurrentTicks();

        // Check if at position
        if (Math.abs(error) <= TurretConstants.POSITION_TOLERANCE) {
            setPower(0);
            // Clear return-to-center flag when we reach center
            if (isReturningToCenter) {
                isReturningToCenter = false;
                hasTarget = false;
            }
            return;
        }

        // PD control
        double p = TurretConstants.kP * error;
        double derivative = (error - lastError) / dt;
        double d = TurretConstants.kD * derivative;
        lastError = error;

        double power = p + d;

        // Add static friction compensation
        if (Math.abs(power) > 0.01) {
            power += Math.signum(power) * TurretConstants.kS;
        }

        // Clamp power
        power = Math.max(-TurretConstants.MAX_POWER,
                Math.min(TurretConstants.MAX_POWER, power));

        setPower(power);
    }

    // ===== VISION AUTO-ALIGN =====

    /**
     * Start vision-based auto-alignment.
     * Call setAlignmentError() with Limelight tx value each loop.
     */
    public void startAutoAlign() {
        isAligning = true;
        hasTarget = false;
        isOdometryTargeting = false;
        isReturningToCenter = false;  // Cancel any return-to-center in progress
        alignError = 0;
        lastAlignError = 0;
    }

    /**
     * Stop auto-alignment
     */
    public void stopAutoAlign() {
        isAligning = false;
        setPower(0);
    }

    /**
     * Set the alignment error from vision (Limelight tx value).
     * Positive tx = target is to the right of center
     */
    public void setAlignmentError(double tx) {
        this.alignError = tx;
    }

    /**
     * Check if turret is aligned to target
     */
    public boolean isAligned() {
        return Math.abs(alignError) <= TurretConstants.ALIGN_DEADBAND;
    }

    /**
     * Vision alignment PID control loop
     */
    private void runAlignmentPID() {
        // Error is negative because we want to turn TOWARDS the target
        // If tx is positive (target to the right), we need positive power
        double error = alignError;

        // Check if aligned (within deadband)
        if (Math.abs(error) <= TurretConstants.ALIGN_DEADBAND) {
            setPower(0);
            return;
        }

        // PD control for alignment
        double p = TurretConstants.ALIGN_kP * error;
        double derivative = error - lastAlignError;
        double d = TurretConstants.ALIGN_kD * derivative;
        lastAlignError = error;

        double power = p + d;

        // Add static friction compensation
        if (Math.abs(power) > 0.01) {
            power += Math.signum(power) * TurretConstants.kS;
        }

        // Clamp power
        power = Math.max(-TurretConstants.MAX_POWER,
                Math.min(TurretConstants.MAX_POWER, power));

        // Check soft limits
        double currentTicks = getCurrentTicks();
        if ((currentTicks >= TurretConstants.MAX_TICKS && power > 0) ||
            (currentTicks <= TurretConstants.MIN_TICKS && power < 0)) {
            power = 0;  // Stop at limits
        }

        setPower(power);
    }

    // ===== ODOMETRY-BASED TARGETING =====

    /**
     * Start odometry-based targeting.
     * Call setOdometryTarget() with robot position and target position each loop.
     * The turret will calculate the required angle to face the target.
     */
    public void startOdometryTargeting() {
        isOdometryTargeting = true;
        isAligning = false;
        hasTarget = false;
        isReturningToCenter = false;  // Cancel any return-to-center in progress
        lastError = 0;
        lastAlignError = 0;  // Reset to prevent derivative spike when switching modes
    }

    /**
     * Stop odometry-based targeting
     */
    public void stopOdometryTargeting() {
        isOdometryTargeting = false;
        setPower(0);
    }

    /**
     * Set odometry target using robot pose and target field position.
     * Calculates the required turret angle to face the target.
     *
     * @param robotX Robot X position on field (inches)
     * @param robotY Robot Y position on field (inches)
     * @param robotHeadingDeg Robot heading in degrees (0 = facing positive X)
     * @param targetX Target X position on field (inches)
     * @param targetY Target Y position on field (inches)
     */
    public void setOdometryTarget(double robotX, double robotY, double robotHeadingDeg,
                                   double targetX, double targetY) {
        this.robotHeading = robotHeadingDeg;

        // Calculate angle from robot to target in field coordinates
        double dx = targetX - robotX;
        double dy = targetY - robotY;

        // Angle to target in field frame (degrees, 0 = positive X axis)
        double fieldAngleToTarget = Math.toDegrees(Math.atan2(dy, dx));

        // Convert to turret angle (relative to robot heading)
        // Turret angle = field angle to target - robot heading
        odometryTargetAngle = fieldAngleToTarget - robotHeadingDeg;

        // Normalize to -180 to 180
        while (odometryTargetAngle > 180) odometryTargetAngle -= 360;
        while (odometryTargetAngle < -180) odometryTargetAngle += 360;

        // Clamp to turret limits
        odometryTargetAngle = TurretConstants.clampAngle(odometryTargetAngle);
    }

    /**
     * Odometry targeting PID control loop
     */
    private void runOdometryPID() {
        // Error is difference between current angle and target angle
        double error = odometryTargetAngle - currentAngle;

        // Normalize error to -180 to 180
        while (error > 180) error -= 360;
        while (error < -180) error += 360;

        // Check if aligned (within deadband)
        if (Math.abs(error) <= TurretConstants.ALIGN_DEADBAND) {
            setPower(0);
            return;
        }

        // PD control
        double p = TurretConstants.ALIGN_kP * error;
        double derivative = error - lastAlignError;
        double d = TurretConstants.ALIGN_kD * derivative;
        lastAlignError = error;

        double power = p + d;

        // Add static friction compensation
        if (Math.abs(power) > 0.01) {
            power += Math.signum(power) * TurretConstants.kS;
        }

        // Clamp power
        power = Math.max(-TurretConstants.MAX_POWER,
                Math.min(TurretConstants.MAX_POWER, power));

        // Check soft limits
        double currentTicks = getCurrentTicks();
        if ((currentTicks >= TurretConstants.MAX_TICKS && power > 0) ||
            (currentTicks <= TurretConstants.MIN_TICKS && power < 0)) {
            power = 0;  // Stop at limits
        }

        setPower(power);
    }

    /**
     * Check if turret is aligned to odometry target
     */
    public boolean isOdometryAligned() {
        double error = Math.abs(odometryTargetAngle - currentAngle);
        return error <= TurretConstants.ALIGN_DEADBAND;
    }

    /**
     * Check if odometry targeting is active
     */
    public boolean isOdometryTargeting() {
        return isOdometryTargeting;
    }

    /**
     * Get the current odometry target angle
     */
    public double getOdometryTargetAngle() {
        return odometryTargetAngle;
    }

    // ===== MANUAL CONTROL =====

    /**
     * Manual spin control (for joystick input)
     */
    public void spin(double power) {
        // Clear all automatic modes when manually controlling
        hasTarget = false;
        isAligning = false;
        isOdometryTargeting = false;
        isReturningToCenter = false;

        // Scale power
        power *= TurretConstants.MANUAL_POWER_SCALE;

        // Check soft limits
        double currentTicks = getCurrentTicks();
        if ((currentTicks >= TurretConstants.MAX_TICKS && power > 0) ||
            (currentTicks <= TurretConstants.MIN_TICKS && power < 0)) {
            power = 0;  // Stop at limits
        }

        setPower(power);
    }

    /**
     * Stop the turret
     */
    public void stop() {
        hasTarget = false;
        isAligning = false;
        isOdometryTargeting = false;
        isReturningToCenter = false;
        setPower(0);
    }

    // ===== STATE QUERIES =====

    /**
     * Check if turret is at target position
     */
    public boolean atPosition() {
        if (!hasTarget) return true;
        double error = Math.abs(targetTicks - getCurrentTicks());
        return error <= TurretConstants.POSITION_TOLERANCE;
    }

    /**
     * Get current turret angle in degrees
     */
    public double getCurrentAngle() {
        return currentAngle;
    }

    /**
     * Get target angle in degrees
     */
    public double getTargetAngle() {
        return targetAngle;
    }

    /**
     * Get current encoder ticks (adjusted for virtual reset)
     */
    public double getCurrentTicks() {
        if (motor == null) return 0;
        return motor.getCurrentPosition() - encoderOffset;
    }

    /**
     * Check if auto-align is active
     */
    public boolean isAutoAligning() {
        return isAligning;
    }

    /**
     * Get current alignment error (from vision)
     */
    public double getAlignmentError() {
        return alignError;
    }

    /**
     * Get current motor power (useful for checking turret direction)
     * Positive = clockwise, Negative = counter-clockwise, ~0 = static
     */
    public double getCurrentPower() {
        return currentPower;
    }

    /**
     * Check if turret is turning clockwise (positive power)
     */
    public boolean isTurningClockwise() {
        return currentPower > 0.05;  // Small threshold to avoid noise
    }

    /**
     * Check if turret is turning counter-clockwise (negative power)
     */
    public boolean isTurningCounterClockwise() {
        return currentPower < -0.05;  // Small threshold to avoid noise
    }

    /**
     * Check if turret is static (not moving)
     */
    public boolean isStatic() {
        return Math.abs(currentPower) <= 0.05;
    }

    /**
     * Check if shooter is allowed to rev based on turret direction.
     * Shooter can only rev when turret is clockwise or static (not counter-clockwise).
     * Exception: Always allow during return-to-center (automatic cable unwinding).
     */
    public boolean canShooterRev() {
        // Always allow shooter during automatic return-to-center
        if (isReturningToCenter) {
            return true;
        }
        // Otherwise, block only when actively turning counter-clockwise
        return !isTurningCounterClockwise();
    }

    // ===== LOW-LEVEL CONTROL =====

    /**
     * Set motor power directly with power efficiency management
     */
    private void setPower(double power) {
        if (motor == null) return;
        this.currentPower = power;

        // Power efficiency: switch to float mode when idle
        if (Math.abs(power) < 0.01) {
            if (!isIdle) {
                motor.floatMode();  // Save power when idle
                isIdle = true;
            }
            motor.setPower(0);
        } else {
            if (isIdle) {
                motor.brakeMode();  // Better control when active
                isIdle = false;
            }
            motor.setPower(power);
        }
    }

    /**
     * Virtual encoder reset
     */
    public void resetEncoder() {
        if (motor != null) {
            encoderOffset = motor.getCurrentPosition();
        }
        currentAngle = 0;
        targetAngle = 0;
    }

    /**
     * Reset turret to center and zero encoder
     */
    public void home() {
        resetEncoder();
        hasTarget = false;
        isAligning = false;
    }

    // ===== TELEMETRY =====

    private void updateTelemetry() {
        try {
            ActiveOpMode.telemetry().addData("--- TURRET ---", "");
            ActiveOpMode.telemetry().addData("Angle", "%.1f deg", currentAngle);
            ActiveOpMode.telemetry().addData("Target", "%.1f deg", targetAngle);
            ActiveOpMode.telemetry().addData("Power", "%.2f", currentPower);

            String mode = "Manual";
            if (isReturningToCenter) mode = "Returning";
            else if (hasTarget) mode = "Position";
            else if (isAligning) mode = "Vision";
            else if (isOdometryTargeting) mode = "Odometry";
            ActiveOpMode.telemetry().addData("Mode", mode);

            // Show turret direction for shooter interlock
            String direction = "Static";
            if (isTurningClockwise()) direction = "CW";
            else if (isTurningCounterClockwise()) direction = "CCW";
            ActiveOpMode.telemetry().addData("Direction", direction);
            ActiveOpMode.telemetry().addData("Shooter OK", canShooterRev() ? "YES" : "NO (CCW)");

            ActiveOpMode.telemetry().addData("At Position", atPosition() ? "YES" : "NO");

            if (isAligning) {
                ActiveOpMode.telemetry().addData("Align Error", "%.2f deg", alignError);
                ActiveOpMode.telemetry().addData("Aligned", isAligned() ? "YES" : "NO");
            }

            if (isOdometryTargeting) {
                ActiveOpMode.telemetry().addData("Odom Target", "%.1f deg", odometryTargetAngle);
                ActiveOpMode.telemetry().addData("Odom Aligned", isOdometryAligned() ? "YES" : "NO");
            }
        } catch (Exception e) {
            // Telemetry not ready
        }
    }

    // ===== COMPONENT REGISTRATION =====

    public SubsystemComponent asCOMPONENT() {
        return new SubsystemComponent(INSTANCE);
    }
}
