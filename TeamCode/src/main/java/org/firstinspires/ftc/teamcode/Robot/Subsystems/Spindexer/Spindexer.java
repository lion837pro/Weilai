package org.firstinspires.ftc.teamcode.Robot.Subsystems.Spindexer;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.NullCommand;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
// import dev.nextftc.hardware.impl.MotorEx;  // COMMENTED OUT - Using servos instead

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drive.VisionConstants;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drive.VisionConstants.BallColor;

/**
 * Spindexer (Spinning Indexer) Subsystem
 *
 * A rotating indexer that holds 3 balls with 6 preset positions.
 * Supports both servo-based (3 servos in gearbox) and motor-based control.
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

    // Hardware - Motor mode (legacy) - COMMENTED OUT, using servos instead
    // private MotorEx motor;

    // Hardware - Servo mode (3 servos in gearbox)
    private Servo servo1;
    private Servo servo2;
    private Servo servo3;
    // private boolean useServos = SpindexerConstants.USE_SERVOS;  // Always using servos now

    // Hardware - Common
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

    // Control state - MOTOR MODE COMMENTED OUT, using servos instead
    // private boolean hasTarget = false;         // Position control active?
    // private double targetTicks = 0;            // Target encoder position
    // private double currentPower = 0;
    // private double encoderOffset = 0;          // Virtual encoder reset offset

    // PID state variables - MOTOR MODE COMMENTED OUT
    // private double integralSum = 0.0;          // Accumulated error for integral term
    // private double lastError = 0.0;            // Previous error for derivative term
    // private long lastPIDTime = 0;              // Last time PID was calculated

    // Timing
    private ElapsedTime moveTimer = new ElapsedTime();

    // Shooter offset state (for mechanical clearance) - Servo mode
    private boolean isInOffsetPosition = false;  // True when spindexer is offset for shooter spin-up
    private double baseShooterServoPos = 0;      // The base shooter position (before offset)

    // Default command
    private Command defaultCommand = new NullCommand();

    @Override
    public void initialize() {
        // Initialize 3 servos in gearbox
        try {
            servo1 = ActiveOpMode.hardwareMap().get(Servo.class, SpindexerConstants.SERVO_1_NAME);
            if (SpindexerConstants.SERVO_1_REVERSED) {
                servo1.setDirection(Servo.Direction.REVERSE);
            }
        } catch (Exception e) {
            servo1 = null;
            ActiveOpMode.telemetry().addData("Servo1 Error", e.getMessage());
        }

        try {
            servo2 = ActiveOpMode.hardwareMap().get(Servo.class, SpindexerConstants.SERVO_2_NAME);
            if (SpindexerConstants.SERVO_2_REVERSED) {
                servo2.setDirection(Servo.Direction.REVERSE);
            }
        } catch (Exception e) {
            servo2 = null;
            ActiveOpMode.telemetry().addData("Servo2 Error", e.getMessage());
        }

        try {
            servo3 = ActiveOpMode.hardwareMap().get(Servo.class, SpindexerConstants.SERVO_3_NAME);
            if (SpindexerConstants.SERVO_3_REVERSED) {
                servo3.setDirection(Servo.Direction.REVERSE);
            }
        } catch (Exception e) {
            servo3 = null;
            ActiveOpMode.telemetry().addData("Servo3 Error", e.getMessage());
        }

        // Set initial position
        setServoPosition(SpindexerConstants.SERVO_POSITIONS[0]);
        isHomed = true;  // Servos don't need homing
        // MOTOR MODE COMMENTED OUT - using servos instead
        // } else {
        //     // Initialize motor (legacy mode)
        //     try {
        //         this.motor = new MotorEx(SpindexerConstants.spindexer);
        //         if (SpindexerConstants.MOTOR_INVERTED) {
        //             motor.reversed();
        //         }
        //         motor.brakeMode();
        //         resetEncoder();  // Virtual encoder reset
        //     } catch (Exception e) {
        //         this.motor = null;
        //         ActiveOpMode.telemetry().addData("Motor Error", e.getMessage());
        //     }
        // }

        // Initialize limit switch (magnetic) - used for motor mode homing
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

            // Configure sensor 1: enable light and set gain
            if (this.colorSensor1 instanceof SwitchableLight) {
                ((SwitchableLight) this.colorSensor1).enableLight(true);
            }
            this.colorSensor1.setGain(SpindexerConstants.COLOR_SENSOR_GAIN);
        } catch (Exception e) {
            this.colorSensor1 = null;
        }

        try {
            this.colorSensor2 = ActiveOpMode.hardwareMap()
                    .get(ColorRangeSensor.class, SpindexerConstants.COLOR_SENSOR_2_NAME);

            // Configure sensor 2: enable light and set gain
            if (this.colorSensor2 instanceof SwitchableLight) {
                ((SwitchableLight) this.colorSensor2).enableLight(true);
            }
            this.colorSensor2.setGain(SpindexerConstants.COLOR_SENSOR_GAIN);
        } catch (Exception e) {
            this.colorSensor2 = null;
        }

        // Initialize ball tracking
        for (int i = 0; i < 3; i++) {
            ballsLoaded[i] = false;
            ballColors[i] = BallColor.UNKNOWN;
        }
    }

    /**
     * Set all servos to a specific position (0.0 to 1.0)
     */
    private void setServoPosition(double position) {
        position = Math.max(0, Math.min(1, position));  // Clamp to valid range
        if (servo1 != null) servo1.setPosition(position);
        if (servo2 != null) servo2.setPosition(position);
        if (servo3 != null) servo3.setPosition(position);
    }

    /**
     * Get average servo position for telemetry
     */
    private double getServoPosition() {
        if (servo1 != null) return servo1.getPosition();
        if (servo2 != null) return servo2.getPosition();
        if (servo3 != null) return servo3.getPosition();
        return 0;
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

        // Servo mode - check if move completed (servos move immediately)
        if (isMoving && moveTimer.milliseconds() >= SpindexerConstants.SETTLE_TIME_MS) {
            isMoving = false;
            currentPosition = targetPosition;
        }
        // MOTOR MODE PID CONTROL COMMENTED OUT - using servos instead
        // } else {
        //     // Motor mode - Position control loop with PID
        //     if (hasTarget && !isSettling) {
        //         long currentTime = System.nanoTime();
        //         double dt = lastPIDTime == 0 ? 0.02 : (currentTime - lastPIDTime) / 1e9;
        //         lastPIDTime = currentTime;
        //
        //         double error = targetTicks - getCurrentTicks();
        //
        //         // Check if at position
        //         if (Math.abs(error) <= SpindexerConstants.POSITION_TOLERANCE) {
        //             // Start settling
        //             isSettling = true;
        //             settleTimer.reset();
        //             setPower(0);
        //             // Reset PID state
        //             integralSum = 0.0;
        //             lastError = 0.0;
        //         } else {
        //             // Full PID control
        //             // Proportional term
        //             double p = SpindexerConstants.kP * error;
        //
        //             // Integral term (with anti-windup)
        //             integralSum += error * dt;
        //             integralSum = Math.max(-50, Math.min(50, integralSum)); // Clamp integral
        //             double i = SpindexerConstants.kI * integralSum;
        //
        //             // Derivative term
        //             double derivative = (error - lastError) / dt;
        //             double d = SpindexerConstants.kD * derivative;
        //             lastError = error;
        //
        //             // Combine PID terms
        //             double power = p + i + d;
        //
        //             // Add static friction compensation
        //             if (Math.abs(power) > 0.01) {
        //                 power += Math.signum(power) * SpindexerConstants.kS;
        //             }
        //
        //             // Clamp power
        //             power = Math.max(-SpindexerConstants.MAX_POWER,
        //                     Math.min(SpindexerConstants.MAX_POWER, power));
        //
        //             setPower(power);
        //         }
        //     }
        //
        //     // Check settle complete
        //     if (isSettling && settleTimer.milliseconds() >= SpindexerConstants.SETTLE_TIME_MS) {
        //         isSettling = false;
        //         isMoving = false;
        //         currentPosition = targetPosition;
        //     }
        // }

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

        // Servo mode - set position directly (always use servos now)
        double servoPos = SpindexerConstants.SERVO_POSITIONS[positionIndex];
        setServoPosition(servoPos);
        isMoving = true;
        moveTimer.reset();

        // MOTOR MODE COMMENTED OUT - using servos instead
        // if (useServos) {
        //     // Servo mode - set position directly
        //     double servoPos = SpindexerConstants.SERVO_POSITIONS[positionIndex];
        //     setServoPosition(servoPos);
        //     isMoving = true;
        //     moveTimer.reset();
        // } else {
        //     // Motor mode - use encoder-based position control
        //     targetTicks = SpindexerConstants.getPositionTicks(positionIndex);
        //
        //     // Optimize rotation direction if enabled
        //     if (SpindexerConstants.OPTIMIZE_ROTATION_DIRECTION) {
        //         targetTicks = optimizeTarget(targetTicks);
        //     }
        //
        //     // Reset PID state for new target
        //     integralSum = 0.0;
        //     lastError = 0.0;
        //     lastPIDTime = 0;
        //
        //     hasTarget = true;
        //     isMoving = true;
        //     isSettling = false;
        //     moveTimer.reset();
        // }
    }

    // MOTOR MODE COMMENTED OUT - optimizeTarget() uses encoder ticks
    // /**
    //  * Optimize target to use shortest rotation path
    //  */
    // private double optimizeTarget(double target) {
    //     double current = getCurrentTicks();
    //     double ticksPerRev = SpindexerConstants.TICKS_PER_SPINDEXER_REV;
    //
    //     // Calculate forward and backward distances
    //     double forwardDist = target - current;
    //     if (forwardDist < 0) forwardDist += ticksPerRev;
    //
    //     double backwardDist = current - target;
    //     if (backwardDist < 0) backwardDist += ticksPerRev;
    //
    //     // Choose shorter path
    //     if (backwardDist < forwardDist) {
    //         return current - backwardDist;
    //     } else {
    //         return current + forwardDist;
    //     }
    // }

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

    // ===== SHOOTER CLEARANCE OFFSET - SERVO MODE =====
    // Offset to move ball away from shooter wheel during spin-up

    /**
     * Calculate servo offset for shooter clearance (60 degrees = 1/6 of full rotation)
     */
    private static final double SERVO_OFFSET = 1.0 / 6.0;  // 60 degrees in servo units

    /**
     * Apply offset to move ball away from shooter wheel during spin-up.
     * Call this BEFORE spinning up the shooter to prevent ball friction.
     * The spindexer should already be at a shooter position.
     */
    public void applyShooterOffset() {
        if (isInOffsetPosition) return;  // Already offset

        baseShooterServoPos = getServoPosition();
        double offsetPos = baseShooterServoPos + SERVO_OFFSET;
        // Wrap around if needed (keep in 0-1 range)
        if (offsetPos > 1.0) offsetPos -= 1.0;

        setServoPosition(offsetPos);
        isMoving = true;
        isInOffsetPosition = true;
        moveTimer.reset();
    }

    /**
     * Remove offset to bring ball back to shooter position for firing.
     * Call this AFTER shooter reaches target RPM.
     */
    public void removeShooterOffset() {
        if (!isInOffsetPosition) return;  // Not offset

        setServoPosition(baseShooterServoPos);
        isMoving = true;
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
        baseShooterServoPos = 0;
    }

    // ===== HOMING =====

    /**
     * Start homing routine (should be run as a command)
     * For servos, this just goes to position 0 immediately.
     */
    public void startHoming() {
        // Servos don't need homing - just go to position 0
        goToPosition(0);
        isHomed = true;

        // MOTOR MODE COMMENTED OUT
        // if (useServos) {
        //     // Servos don't need homing - just go to position 0
        //     goToPosition(0);
        //     isHomed = true;
        // } else {
        //     // Motor mode - use limit switch
        //     isHomed = false;
        //     hasTarget = false;
        //     setPower(-SpindexerConstants.HOMING_POWER);  // Rotate until limit switch
        //     moveTimer.reset();
        // }
    }

    /**
     * Check if limit switch is triggered (at home position)
     * For servos, always returns true if at position 0.
     */
    public boolean isAtHome() {
        // Servos - just check if at position 0
        return currentPosition == 0 && !isMoving;

        // MOTOR MODE COMMENTED OUT
        // if (useServos) {
        //     return currentPosition == 0 && !isMoving;
        // }
        //
        // if (limitSwitch == null) return false;
        //
        // boolean state = limitSwitch.getState();
        // // Apply polarity from constants
        // if (SpindexerConstants.LIMIT_SWITCH_ACTIVE_LOW) {
        //     return !state;  // Active-low: triggered when state is LOW (false)
        // } else {
        //     return state;   // Active-high: triggered when state is HIGH (true)
        // }
    }

    /**
     * Get raw limit switch state for debugging (true = HIGH, false = LOW)
     */
    public boolean getLimitSwitchRawState() {
        if (limitSwitch == null) return false;
        return limitSwitch.getState();
    }

    /**
     * Complete homing routine
     */
    public void finishHoming() {
        // Servos are already homed
        currentPosition = 0;
        targetPosition = 0;
        isHomed = true;
        isMoving = false;

        // MOTOR MODE COMMENTED OUT
        // if (useServos) {
        //     // Servos are already homed
        //     currentPosition = 0;
        //     targetPosition = 0;
        //     isHomed = true;
        //     isMoving = false;
        // } else {
        //     // Motor mode
        //     setPower(0);
        //     resetEncoder();  // Virtual encoder reset
        //     currentPosition = 0;
        //     targetPosition = 0;
        //     isHomed = true;
        //     hasTarget = false;
        //     isMoving = false;
        // }
    }

    // ===== BALL DETECTION AND COLOR =====

    /**
     * Update ball detection and color from color sensors
     */
    private void updateBallDetection() {
        if (colorSensor1 == null) return;

        // Only detect when spindexer is generally aligned to intake
        // We relax the strict atPosition() check to allow detection while settling
        // as long as we are in an intake position (0, 2, 4)
        if (!isAtIntakePosition()) return;

        // Optional: Check if we are "close enough" to position if not strictly atPosition
        // But for now, if we are in Intake Mode (even if oscillating slightly), we want to detect.

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
     * Force check for ball at current slot (for manual detection)
     * Returns true if ball was detected and added
     */
    public boolean forceCheckBall() {
        if (colorSensor1 == null || !isAtIntakePosition()) return false;

        int slot = currentPosition / 2;
        if (ballsLoaded[slot]) return false; // Already has ball

        double distance = colorSensor1.getDistance(DistanceUnit.MM);
        boolean ballDetected = distance < SpindexerConstants.COLOR_PROXIMITY_THRESHOLD;

        if (ballDetected) {
            NormalizedRGBA colors = colorSensor1.getNormalizedColors();
            int red = (int)(colors.red * 255);
            int green = (int)(colors.green * 255);
            int blue = (int)(colors.blue * 255);

            lastDetectedRed = red;
            lastDetectedGreen = green;
            lastDetectedBlue = blue;

            BallColor detectedColor = SpindexerConstants.detectBallColor(red, green, blue);
            lastDetectedColor = detectedColor;

            ballsLoaded[slot] = true;
            ballColors[slot] = detectedColor;
            return true;
        }
        return false;
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
        // Servo mode - just check if not moving
        return !isMoving;

        // MOTOR MODE COMMENTED OUT
        // if (!hasTarget) return true;
        // return !isMoving && !isSettling;
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
     * In servo mode, returns position equivalent based on servo position.
     */
    public double getCurrentTicks() {
        // Servo mode - convert servo position to equivalent ticks
        return getServoPosition() * SpindexerConstants.TICKS_PER_SPINDEXER_REV;

        // MOTOR MODE COMMENTED OUT
        // if (useServos) {
        //     // Convert servo position to equivalent ticks
        //     return getServoPosition() * SpindexerConstants.TICKS_PER_SPINDEXER_REV;
        // }
        // if (motor == null) return 0;
        // return motor.getCurrentPosition() - encoderOffset;
    }

    // MOTOR MODE COMMENTED OUT - resetEncoder() uses motor
    // /**
    //  * Virtual encoder reset - stores current position as offset
    //  */
    // private void resetEncoder() {
    //     if (motor != null && !useServos) {
    //         encoderOffset = motor.getCurrentPosition();
    //     }
    // }

    // MOTOR MODE COMMENTED OUT - getVelocity() uses motor
    // /**
    //  * Get current motor velocity
    //  * In servo mode, returns 0 (servos don't report velocity).
    //  */
    // public double getVelocity() {
    //     if (useServos || motor == null) return 0;
    //     return motor.getVelocity();
    // }

    /**
     * Get the last detected color (from most recent ball intake)
     */
    public BallColor getLastDetectedColor() {
        return lastDetectedColor;
    }

    // ===== LOW-LEVEL CONTROL =====

    // MOTOR MODE COMMENTED OUT - setPower() uses motor
    // /**
    //  * Set motor power directly (for manual control or homing)
    //  * Only works in motor mode.
    //  */
    // public void setPower(double power) {
    //     if (useServos) return;  // Servos don't support power control
    //     if (motor == null) return;
    //     this.currentPower = power;
    //     motor.setPower(power);
    // }

    /**
     * Stop the spindexer
     */
    public void stop() {
        isMoving = false;
        // Servos hold their position automatically

        // MOTOR MODE COMMENTED OUT
        // hasTarget = false;
        // isMoving = false;
        // isSettling = false;
        // if (!useServos) {
        //     setPower(0);
        // }
    }

    /**
     * Manual spin (for testing or clearing jams)
     * In servo mode, this adjusts the servo position incrementally.
     */
    public void spin(double power) {
        // Servo mode - use power as a position delta
        if (Math.abs(power) > 0.1) {
            double currentPos = getServoPosition();
            double delta = power * 0.01;  // Small increment per loop
            setServoPosition(currentPos + delta);
        }

        // MOTOR MODE COMMENTED OUT
        // if (useServos) {
        //     // In servo mode, use power as a position delta
        //     if (Math.abs(power) > 0.1) {
        //         double currentPos = getServoPosition();
        //         double delta = power * 0.01;  // Small increment per loop
        //         setServoPosition(currentPos + delta);
        //     }
        // } else {
        //     hasTarget = false;
        //     isMoving = false;
        //     setPower(power);
        // }
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

            // Limit switch debugging - show both raw state and interpreted value
            if (limitSwitch != null) {
                boolean rawState = limitSwitch.getState();
                ActiveOpMode.telemetry().addData("Limit Switch Raw", rawState ? "HIGH" : "LOW");
                ActiveOpMode.telemetry().addData("Limit Switch", isAtHome() ? "TRIGGERED" : "Open");
            } else {
                ActiveOpMode.telemetry().addData("Limit Switch", "NOT FOUND");
            }

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

            // Real-time sensor readings for debugging
            if (colorSensor1 != null) {
                try {
                    NormalizedRGBA live1 = colorSensor1.getNormalizedColors();
                    double dist1 = colorSensor1.getDistance(DistanceUnit.MM);
                    ActiveOpMode.telemetry().addData("Sensor 1", "R:%.2f G:%.2f B:%.2f Dist:%.1fmm",
                            live1.red, live1.green, live1.blue, dist1);
                } catch (Exception e) {
                    ActiveOpMode.telemetry().addData("Sensor 1", "ERROR: %s", e.getMessage());
                }
            } else {
                ActiveOpMode.telemetry().addData("Sensor 1", "NOT FOUND");
            }

            if (colorSensor2 != null) {
                try {
                    NormalizedRGBA live2 = colorSensor2.getNormalizedColors();
                    double dist2 = colorSensor2.getDistance(DistanceUnit.MM);
                    ActiveOpMode.telemetry().addData("Sensor 2", "R:%.2f G:%.2f B:%.2f Dist:%.1fmm",
                            live2.red, live2.green, live2.blue, dist2);
                } catch (Exception e) {
                    ActiveOpMode.telemetry().addData("Sensor 2", "ERROR: %s", e.getMessage());
                }
            } else {
                ActiveOpMode.telemetry().addData("Sensor 2", "NOT FOUND");
            }

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
