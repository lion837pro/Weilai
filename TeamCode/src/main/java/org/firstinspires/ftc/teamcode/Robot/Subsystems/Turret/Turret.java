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
 * Supports both position control and vision-based auto-alignment.
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

    // Control state
    private double targetTicks = 0;           // Target encoder position
    private double currentPower = 0;
    private double encoderOffset = 0;         // Virtual encoder reset offset

    // PID state
    private double lastError = 0.0;
    private long lastPIDTime = 0;

    // Alignment state (for vision-based auto-align)
    private double alignError = 0;            // Error from vision (tx)
    private double lastAlignError = 0;

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

        // Position control loop
        if (hasTarget && !isAligning) {
            runPositionPID();
        }

        // Vision alignment loop (separate from position control)
        if (isAligning) {
            runAlignmentPID();
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

        hasTarget = true;
        isAligning = false;
        moveTimer.reset();
    }

    /**
     * Move turret to center position (0 degrees)
     */
    public void goToCenter() {
        goToAngle(TurretConstants.POSITION_CENTER);
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

    // ===== MANUAL CONTROL =====

    /**
     * Manual spin control (for joystick input)
     */
    public void spin(double power) {
        hasTarget = false;
        isAligning = false;

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

    // ===== LOW-LEVEL CONTROL =====

    /**
     * Set motor power directly
     */
    private void setPower(double power) {
        if (motor == null) return;
        this.currentPower = power;
        motor.setPower(power);
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
            ActiveOpMode.telemetry().addData("Mode", hasTarget ? "Position" : (isAligning ? "Auto-Align" : "Manual"));
            ActiveOpMode.telemetry().addData("At Position", atPosition() ? "YES" : "NO");

            if (isAligning) {
                ActiveOpMode.telemetry().addData("Align Error", "%.2f deg", alignError);
                ActiveOpMode.telemetry().addData("Aligned", isAligned() ? "YES" : "NO");
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
