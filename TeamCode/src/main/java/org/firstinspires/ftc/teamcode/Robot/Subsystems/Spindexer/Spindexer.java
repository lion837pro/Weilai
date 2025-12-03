package org.firstinspires.ftc.teamcode.Robot.Subsystems.Spindexer;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.NullCommand;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.hardware.HardwareManager;
import dev.nextftc.hardware.impl.MotorEx;

import org.firstinspires.ftc.teamcode.Lib.STZLite.Math.Intervals.Interval;

/**
 * Spindexer (Spinning Indexer) Subsystem
 *
 * A rotating indexer that holds 3 balls with 6 preset positions.
 * Uses a magnetic limit switch for homing and 2 color sensors for ball detection.
 *
 * Slot mapping:
 * - Slot 0: Positions 0 (intake) and 1 (shooter)
 * - Slot 1: Positions 2 (intake) and 3 (shooter)
 * - Slot 2: Positions 4 (intake) and 5 (shooter)
 */
public class Spindexer implements Subsystem {

    public static final Spindexer INSTANCE = new Spindexer();

    // Hardware
    private MotorEx motor;
    private DigitalChannel limitSwitch;
    private ColorRangeSensor colorSensor1;
    private ColorRangeSensor colorSensor2;

    // State tracking
    private int currentPosition = 0;           // Current position index (0-5)
    private int targetPosition = 0;            // Target position index (0-5)
    private boolean isHomed = false;           // Has the spindexer been homed?
    private boolean isMoving = false;          // Is the spindexer currently moving?
    private boolean[] ballsLoaded = new boolean[3];  // Track balls in each slot (0, 1, 2)

    // Control state
    private boolean hasTarget = false;         // Position control active?
    private double targetTicks = 0;            // Target encoder position
    private double currentPower = 0;

    // Timing
    private ElapsedTime moveTimer = new ElapsedTime();
    private ElapsedTime settleTimer = new ElapsedTime();
    private boolean isSettling = false;

    // Default command
    private Command defaultCommand = new NullCommand();

    @Override
    public void initialize() {
        // Initialize motor
        this.motor = new MotorEx(SpindexerConstants.MOTOR_NAME);
        if (SpindexerConstants.MOTOR_INVERTED) {
            motor.reversed();
        }
        motor.brakeMode();
        motor.resetEncoder();

        // Initialize limit switch (magnetic)
        try {
            this.limitSwitch = HardwareManager.getHardwareMap()
                    .get(DigitalChannel.class, SpindexerConstants.LIMIT_SWITCH_NAME);
            limitSwitch.setMode(DigitalChannel.Mode.INPUT);
        } catch (Exception e) {
            this.limitSwitch = null;
        }

        // Initialize color sensors
        try {
            this.colorSensor1 = HardwareManager.getHardwareMap()
                    .get(ColorRangeSensor.class, SpindexerConstants.COLOR_SENSOR_1_NAME);
        } catch (Exception e) {
            this.colorSensor1 = null;
        }

        try {
            this.colorSensor2 = HardwareManager.getHardwareMap()
                    .get(ColorRangeSensor.class, SpindexerConstants.COLOR_SENSOR_2_NAME);
        } catch (Exception e) {
            this.colorSensor2 = null;
        }

        // Initialize ball tracking
        for (int i = 0; i < 3; i++) {
            ballsLoaded[i] = false;
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
        // Update ball detection from color sensors
        updateBallDetection();

        // Position control loop
        if (hasTarget && !isSettling) {
            double error = targetTicks - getCurrentTicks();

            // Check if at position
            if (Math.abs(error) <= SpindexerConstants.POSITION_TOLERANCE) {
                // Start settling
                isSettling = true;
                settleTimer.reset();
                setPower(0);
            } else {
                // PID control
                double power = error * SpindexerConstants.kP;

                // Add static friction compensation
                if (Math.abs(power) > 0.01) {
                    power += Math.signum(power) * SpindexerConstants.kS;
                }

                // Clamp power
                power = Math.max(-SpindexerConstants.MAX_POWER,
                        Math.min(SpindexerConstants.MAX_POWER, power));

                setPower(power);
            }
        }

        // Check settle complete
        if (isSettling && settleTimer.milliseconds() >= SpindexerConstants.SETTLE_TIME_MS) {
            isSettling = false;
            isMoving = false;
            currentPosition = targetPosition;
        }

        // Update telemetry
        if (SpindexerConstants.ENABLE_TELEMETRY) {
            updateTelemetry();
        }
    }

    // ===== POSITION CONTROL METHODS =====

    /**
     * Move to a specific position index (0-5)
     */
    public void goToPosition(int positionIndex) {
        if (positionIndex < 0 || positionIndex >= SpindexerConstants.POSITION_COUNT) {
            return;
        }

        targetPosition = positionIndex;
        targetTicks = SpindexerConstants.getPositionTicks(positionIndex);

        // Optimize rotation direction if enabled
        if (SpindexerConstants.OPTIMIZE_ROTATION_DIRECTION) {
            targetTicks = optimizeTarget(targetTicks);
        }

        hasTarget = true;
        isMoving = true;
        isSettling = false;
        moveTimer.reset();
    }

    /**
     * Optimize target to use shortest rotation path
     */
    private double optimizeTarget(double target) {
        double current = getCurrentTicks();
        double ticksPerRev = SpindexerConstants.TICKS_PER_SPINDEXER_REV;

        // Calculate forward and backward distances
        double forwardDist = target - current;
        if (forwardDist < 0) forwardDist += ticksPerRev;

        double backwardDist = current - target;
        if (backwardDist < 0) backwardDist += ticksPerRev;

        // Choose shorter path
        if (backwardDist < forwardDist) {
            return current - backwardDist;
        } else {
            return current + forwardDist;
        }
    }

    /**
     * Move to the next intake position (empty slot)
     */
    public void goToNextIntakePosition() {
        for (int slot = 0; slot < SpindexerConstants.SLOTS_COUNT; slot++) {
            if (!ballsLoaded[slot]) {
                int intakePos = SpindexerConstants.getIntakePosition(slot);
                goToPosition(intakePos);
                return;
            }
        }
        // All slots full, stay in place
    }

    /**
     * Move to the next shooter position (loaded slot)
     */
    public void goToNextShooterPosition() {
        for (int slot = 0; slot < SpindexerConstants.SLOTS_COUNT; slot++) {
            if (ballsLoaded[slot]) {
                int shooterPos = SpindexerConstants.getShooterPosition(slot);
                goToPosition(shooterPos);
                return;
            }
        }
        // All slots empty, stay in place
    }

    /**
     * Index forward by one position (60 degrees)
     */
    public void indexForward() {
        int nextPos = (currentPosition + 1) % SpindexerConstants.POSITION_COUNT;
        goToPosition(nextPos);
    }

    /**
     * Index backward by one position (60 degrees)
     */
    public void indexBackward() {
        int prevPos = (currentPosition - 1 + SpindexerConstants.POSITION_COUNT)
                % SpindexerConstants.POSITION_COUNT;
        goToPosition(prevPos);
    }

    // ===== HOMING =====

    /**
     * Start homing routine (should be run as a command)
     */
    public void startHoming() {
        isHomed = false;
        hasTarget = false;
        setPower(-SpindexerConstants.HOMING_POWER);  // Rotate until limit switch
        moveTimer.reset();
    }

    /**
     * Check if limit switch is triggered (at home position)
     */
    public boolean isAtHome() {
        if (limitSwitch == null) return false;
        return !limitSwitch.getState();  // Usually active-low
    }

    /**
     * Complete homing routine
     */
    public void finishHoming() {
        setPower(0);
        motor.resetEncoder();
        currentPosition = 0;
        targetPosition = 0;
        isHomed = true;
        hasTarget = false;
        isMoving = false;
    }

    // ===== BALL DETECTION =====

    /**
     * Update ball detection from color sensors
     */
    private void updateBallDetection() {
        // Color sensor 1 detects ball at current slot when at intake position
        if (colorSensor1 != null && isAtIntakePosition()) {
            int slot = currentPosition / 2;  // 0->0, 2->1, 4->2
            boolean detected = detectBallAtSensor(colorSensor1);
            if (detected && !ballsLoaded[slot]) {
                ballsLoaded[slot] = true;  // Ball loaded
            }
        }

        // Color sensor 2 can be used for additional detection or verification
        // Implementation depends on physical sensor placement
    }

    /**
     * Check if a ball is detected at a color sensor
     */
    private boolean detectBallAtSensor(ColorRangeSensor sensor) {
        if (sensor == null) return false;

        // Use proximity/distance for detection
        double proximity = sensor.getDistance(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.MM);
        return proximity < SpindexerConstants.COLOR_PROXIMITY_THRESHOLD;
    }

    /**
     * Manually mark a slot as having a ball
     */
    public void setBallLoaded(int slot, boolean loaded) {
        if (slot >= 0 && slot < SpindexerConstants.SLOTS_COUNT) {
            ballsLoaded[slot] = loaded;
        }
    }

    /**
     * Mark current shooter slot as empty (after shooting)
     */
    public void markCurrentSlotEmpty() {
        if (isAtShooterPosition()) {
            int slot = (currentPosition - 1) / 2;  // 1->0, 3->1, 5->2
            ballsLoaded[slot] = false;
        }
    }

    // ===== STATE QUERIES =====

    /**
     * Check if at target position
     */
    public boolean atPosition() {
        if (!hasTarget) return true;
        return !isMoving && !isSettling;
    }

    /**
     * Check if currently at an intake position
     */
    public boolean isAtIntakePosition() {
        return currentPosition % 2 == 0;  // Positions 0, 2, 4
    }

    /**
     * Check if currently at a shooter position
     */
    public boolean isAtShooterPosition() {
        return currentPosition % 2 == 1;  // Positions 1, 3, 5
    }

    /**
     * Check if a specific slot has a ball
     */
    public boolean hasBall(int slot) {
        if (slot < 0 || slot >= SpindexerConstants.SLOTS_COUNT) return false;
        return ballsLoaded[slot];
    }

    /**
     * Get total number of balls loaded
     */
    public int getBallCount() {
        int count = 0;
        for (boolean loaded : ballsLoaded) {
            if (loaded) count++;
        }
        return count;
    }

    /**
     * Check if spindexer is full
     */
    public boolean isFull() {
        return getBallCount() >= SpindexerConstants.SLOTS_COUNT;
    }

    /**
     * Check if spindexer is empty
     */
    public boolean isEmpty() {
        return getBallCount() == 0;
    }

    /**
     * Check if spindexer has been homed
     */
    public boolean isHomed() {
        return isHomed;
    }

    /**
     * Get current position index
     */
    public int getCurrentPosition() {
        return currentPosition;
    }

    /**
     * Get current encoder ticks
     */
    public double getCurrentTicks() {
        return motor.getCurrentPosition();
    }

    /**
     * Get current motor velocity
     */
    public double getVelocity() {
        return motor.getVelocity();
    }

    // ===== LOW-LEVEL CONTROL =====

    /**
     * Set motor power directly (for manual control or homing)
     */
    public void setPower(double power) {
        this.currentPower = power;
        motor.setPower(power);
    }

    /**
     * Stop the spindexer
     */
    public void stop() {
        hasTarget = false;
        isMoving = false;
        isSettling = false;
        setPower(0);
    }

    /**
     * Manual spin (for testing or clearing jams)
     */
    public void spin(double power) {
        hasTarget = false;
        isMoving = false;
        setPower(power);
    }

    // ===== TELEMETRY =====

    private void updateTelemetry() {
        try {
            ActiveOpMode.telemetry().addData("--- SPINDEXER ---", "");
            ActiveOpMode.telemetry().addData("Homed", isHomed ? "YES" : "NO");
            ActiveOpMode.telemetry().addData("Position", "%d (%s)",
                    currentPosition, isAtIntakePosition() ? "INTAKE" : "SHOOTER");
            ActiveOpMode.telemetry().addData("Target Position", targetPosition);
            ActiveOpMode.telemetry().addData("Encoder Ticks", "%.1f", getCurrentTicks());
            ActiveOpMode.telemetry().addData("Target Ticks", "%.1f", targetTicks);
            ActiveOpMode.telemetry().addData("Error", "%.1f", targetTicks - getCurrentTicks());
            ActiveOpMode.telemetry().addData("Power", "%.2f", currentPower);
            ActiveOpMode.telemetry().addData("At Position", atPosition() ? "YES" : "NO");
            ActiveOpMode.telemetry().addData("Limit Switch", isAtHome() ? "TRIGGERED" : "Open");

            // Ball status
            ActiveOpMode.telemetry().addData("Balls", "[%s %s %s] = %d",
                    ballsLoaded[0] ? "●" : "○",
                    ballsLoaded[1] ? "●" : "○",
                    ballsLoaded[2] ? "●" : "○",
                    getBallCount());

        } catch (Exception e) {
            // Failsafe if telemetry isn't ready
        }
    }

    // ===== COMPONENT REGISTRATION =====

    public SubsystemComponent asCOMPONENT() {
        return new SubsystemComponent(INSTANCE);
    }
}
