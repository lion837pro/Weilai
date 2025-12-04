package org.firstinspires.ftc.teamcode.Robot.Subsystems.Spindexer;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.NullCommand;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.MotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drive.VisionConstants;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drive.VisionConstants.BallColor;

/**
 * Spindexer (Spinning Indexer) Subsystem
 *
 * A rotating indexer that holds 3 balls with 6 preset positions.
 * Uses a magnetic limit switch for homing and 2 color sensors for ball detection.
 * Tracks ball colors (GREEN or PURPLE) for automatic color sorting with AprilTags.
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

    // Ball color tracking (replaces simple boolean tracking)
    private BallColor[] ballColors = new BallColor[3];  // Color of ball in each slot
    private boolean[] ballsLoaded = new boolean[3];     // Whether slot has a ball

    // Last detected color (for intake)
    private BallColor lastDetectedColor = BallColor.UNKNOWN;
    private int lastDetectedRed = 0;
    private int lastDetectedGreen = 0;
    private int lastDetectedBlue = 0;

    // Control state
    private boolean hasTarget = false;         // Position control active?
    private double targetTicks = 0;            // Target encoder position
    private double currentPower = 0;
    private double encoderOffset = 0;          // Virtual encoder reset offset

    // Shooter offset state (for mechanical clearance)
    private boolean isInOffsetPosition = false;  // True when spindexer is offset for shooter spin-up
    private double baseShooterTicks = 0;         // The base shooter position (before offset)

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
        resetEncoder();  // Virtual encoder reset

        // Initialize limit switch (magnetic)
        try {
            this.limitSwitch = ActiveOpMode.hardwareMap()
                    .get(DigitalChannel.class, SpindexerConstants.LIMIT_SWITCH_NAME);
            limitSwitch.setMode(DigitalChannel.Mode.INPUT);
        } catch (Exception e) {
            this.limitSwitch = null;
        }

        // Initialize color sensors
        try {
            this.colorSensor1 = ActiveOpMode.hardwareMap()
                    .get(ColorRangeSensor.class, SpindexerConstants.COLOR_SENSOR_1_NAME);
        } catch (Exception e) {
            this.colorSensor1 = null;
        }

        try {
            this.colorSensor2 = ActiveOpMode.hardwareMap()
                    .get(ColorRangeSensor.class, SpindexerConstants.COLOR_SENSOR_2_NAME);
        } catch (Exception e) {
            this.colorSensor2 = null;
        }

        // Initialize ball tracking
        for (int i = 0; i < 3; i++) {
            ballsLoaded[i] = false;
            ballColors[i] = BallColor.UNKNOWN;
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
        // Update ball detection and color from color sensors
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
     * Move to shooter position for a specific ball color.
     * Finds the first slot with that color and goes to its shooter position.
     * Returns true if found, false if no ball of that color exists.
     */
    public boolean goToShooterPositionForColor(BallColor targetColor) {
        for (int slot = 0; slot < SpindexerConstants.SLOTS_COUNT; slot++) {
            if (ballsLoaded[slot] && ballColors[slot] == targetColor) {
                int shooterPos = SpindexerConstants.getShooterPosition(slot);
                goToPosition(shooterPos);
                return true;
            }
        }
        return false;
    }

    /**
     * Move to shooter position for the slot that matches the AprilTag pattern.
     * For tag 21 (G,P,P): shoots slot 0 first (green)
     * For tag 22 (P,G,P): shoots slot 1 first (green)
     * For tag 23 (P,P,G): shoots slot 2 first (green)
     */
    public boolean goToShooterPositionForTag(int tagId) {
        int greenSlot = VisionConstants.getGreenSlotForTag(tagId);
        if (greenSlot == -1) return false;

        // First priority: shoot the GREEN ball at the correct position
        if (ballsLoaded[greenSlot] && ballColors[greenSlot] == BallColor.GREEN) {
            goToPosition(SpindexerConstants.getShooterPosition(greenSlot));
            return true;
        }

        // If no green at correct slot, shoot any loaded ball
        if (getBallCount() > 0) {
            goToNextShooterPosition();
            return true;
        }
        return false;
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

    // ===== SHOOTER CLEARANCE OFFSET =====

    /**
     * Apply offset to move ball away from shooter wheel during spin-up.
     * Call this BEFORE spinning up the shooter to prevent ball friction.
     * The spindexer should already be at a shooter position.
     */
    public void applyShooterOffset() {
        if (isInOffsetPosition) return;  // Already offset

        baseShooterTicks = getCurrentTicks();
        targetTicks = baseShooterTicks + SpindexerConstants.SHOOTER_CLEARANCE_OFFSET_TICKS;
        hasTarget = true;
        isMoving = true;
        isSettling = false;
        isInOffsetPosition = true;
        moveTimer.reset();
    }

    /**
     * Remove offset to bring ball back to shooter position for firing.
     * Call this AFTER shooter reaches target RPM.
     */
    public void removeShooterOffset() {
        if (!isInOffsetPosition) return;  // Not offset

        targetTicks = baseShooterTicks;
        hasTarget = true;
        isMoving = true;
        isSettling = false;
        isInOffsetPosition = false;
        moveTimer.reset();
    }

    /**
     * Check if spindexer is in offset position
     */
    public boolean isInOffsetPosition() {
        return isInOffsetPosition;
    }

    /**
     * Reset offset state (call when starting a new shooting sequence)
     */
    public void resetOffsetState() {
        isInOffsetPosition = false;
        baseShooterTicks = 0;
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
        resetEncoder();  // Virtual encoder reset
        currentPosition = 0;
        targetPosition = 0;
        isHomed = true;
        hasTarget = false;
        isMoving = false;
    }

    // ===== BALL DETECTION AND COLOR =====

    /**
     * Update ball detection and color from color sensors
     */
    private void updateBallDetection() {
        if (colorSensor1 == null) return;

        // Only detect at intake position
        if (!isAtIntakePosition()) return;

        int slot = currentPosition / 2;  // 0->0, 2->1, 4->2

        // Check proximity for ball presence
        double distance = colorSensor1.getDistance(DistanceUnit.MM);
        boolean ballDetected = distance < SpindexerConstants.COLOR_PROXIMITY_THRESHOLD;

        if (ballDetected && !ballsLoaded[slot]) {
            // New ball detected - read its color
            NormalizedRGBA colors = colorSensor1.getNormalizedColors();

            // Convert normalized (0-1) to 0-255 range
            int red = (int)(colors.red * 255);
            int green = (int)(colors.green * 255);
            int blue = (int)(colors.blue * 255);

            // Store for telemetry
            lastDetectedRed = red;
            lastDetectedGreen = green;
            lastDetectedBlue = blue;

            // Determine color
            BallColor detectedColor = SpindexerConstants.detectBallColor(red, green, blue);
            lastDetectedColor = detectedColor;

            // Store ball info
            ballsLoaded[slot] = true;
            ballColors[slot] = detectedColor;
        }
    }

    /**
     * Manually set a ball's color in a slot
     */
    public void setBallColor(int slot, BallColor color) {
        if (slot >= 0 && slot < SpindexerConstants.SLOTS_COUNT) {
            ballColors[slot] = color;
            ballsLoaded[slot] = (color != BallColor.UNKNOWN);
        }
    }

    /**
     * Manually mark a slot as having a ball with specified color
     */
    public void setBallLoaded(int slot, boolean loaded, BallColor color) {
        if (slot >= 0 && slot < SpindexerConstants.SLOTS_COUNT) {
            ballsLoaded[slot] = loaded;
            ballColors[slot] = loaded ? color : BallColor.UNKNOWN;
        }
    }

    /**
     * Manually mark a slot as having a ball (unknown color)
     */
    public void setBallLoaded(int slot, boolean loaded) {
        setBallLoaded(slot, loaded, BallColor.UNKNOWN);
    }

    /**
     * Mark current shooter slot as empty (after shooting)
     */
    public void markCurrentSlotEmpty() {
        if (isAtShooterPosition()) {
            int slot = (currentPosition - 1) / 2;  // 1->0, 3->1, 5->2
            ballsLoaded[slot] = false;
            ballColors[slot] = BallColor.UNKNOWN;
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
     * Get the color of the ball in a slot
     */
    public BallColor getBallColor(int slot) {
        if (slot < 0 || slot >= SpindexerConstants.SLOTS_COUNT) return BallColor.UNKNOWN;
        return ballColors[slot];
    }

    /**
     * Get all ball colors as an array
     */
    public BallColor[] getAllBallColors() {
        return ballColors.clone();
    }

    /**
     * Check if we have a ball of a specific color
     */
    public boolean hasBallOfColor(BallColor color) {
        for (int i = 0; i < SpindexerConstants.SLOTS_COUNT; i++) {
            if (ballsLoaded[i] && ballColors[i] == color) {
                return true;
            }
        }
        return false;
    }

    /**
     * Count balls of a specific color
     */
    public int countBallsOfColor(BallColor color) {
        int count = 0;
        for (int i = 0; i < SpindexerConstants.SLOTS_COUNT; i++) {
            if (ballsLoaded[i] && ballColors[i] == color) {
                count++;
            }
        }
        return count;
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
     * Get current encoder ticks (adjusted for virtual reset)
     */
    public double getCurrentTicks() {
        return motor.getCurrentPosition() - encoderOffset;
    }

    /**
     * Virtual encoder reset - stores current position as offset
     */
    private void resetEncoder() {
        encoderOffset = motor.getCurrentPosition();
    }

    /**
     * Get current motor velocity
     */
    public double getVelocity() {
        return motor.getVelocity();
    }

    /**
     * Get the last detected color (from most recent ball intake)
     */
    public BallColor getLastDetectedColor() {
        return lastDetectedColor;
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
            ActiveOpMode.telemetry().addData("At Position", atPosition() ? "YES" : "NO");
            ActiveOpMode.telemetry().addData("Limit Switch", isAtHome() ? "TRIGGERED" : "Open");

            // Ball status with colors
            String slot0 = ballsLoaded[0] ? colorToSymbol(ballColors[0]) : "○";
            String slot1 = ballsLoaded[1] ? colorToSymbol(ballColors[1]) : "○";
            String slot2 = ballsLoaded[2] ? colorToSymbol(ballColors[2]) : "○";
            ActiveOpMode.telemetry().addData("Balls", "[%s %s %s] = %d",
                    slot0, slot1, slot2, getBallCount());

            // Color counts
            ActiveOpMode.telemetry().addData("Green/Purple",
                    "%d G, %d P",
                    countBallsOfColor(BallColor.GREEN),
                    countBallsOfColor(BallColor.PURPLE));

            // Last detected color info
            ActiveOpMode.telemetry().addData("Last Color", "%s (R:%d G:%d B:%d)",
                    lastDetectedColor.toString(),
                    lastDetectedRed, lastDetectedGreen, lastDetectedBlue);

        } catch (Exception e) {
            // Failsafe if telemetry isn't ready
        }
    }

    private String colorToSymbol(BallColor color) {
        switch (color) {
            case GREEN: return "G";
            case PURPLE: return "P";
            default: return "?";
        }
    }

    // ===== COMPONENT REGISTRATION =====

    public SubsystemComponent asCOMPONENT() {
        return new SubsystemComponent(INSTANCE);
    }
}
