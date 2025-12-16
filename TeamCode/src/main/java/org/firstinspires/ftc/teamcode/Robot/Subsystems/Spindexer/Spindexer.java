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

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drive.VisionConstants;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drive.VisionConstants.BallColor;

/**
 * Spindexer (Spinning Indexer) Subsystem
 *
 * A rotating indexer that holds 3 balls with 6 preset positions.
 * Uses 3 GoBilda continuous rotation servos in a gearbox configuration.
 * Uses a magnetic limit switch for homing and 2 color sensors for ball detection.
 * Tracks ball colors (GREEN or PURPLE) for automatic color sorting with AprilTags.
 *
 * CONTINUOUS ROTATION SERVO CONTROL:
 * - Servos act like motors: 0.5 = stop, 0.0 = full reverse, 1.0 = full forward
 * - Position is tracked virtually using time-based movement
 * - Limit switch is used for homing to establish position 0
 *
 * Slot mapping:
 * - Slot 0: Positions 0 (intake) and 1 (shooter)
 * - Slot 1: Positions 2 (intake) and 3 (shooter)
 * - Slot 2: Positions 4 (intake) and 5 (shooter)
 */
public class Spindexer implements Subsystem {

    public static final Spindexer INSTANCE = new Spindexer();

    // Hardware - 3 continuous rotation servos in gearbox
    private Servo servo1;
    private Servo servo2;
    private Servo servo3;

    // Hardware - Common
    private DigitalChannel limitSwitch;
    private ColorRangeSensor colorSensor1;
    private ColorRangeSensor colorSensor2;

    // State tracking
    private int currentPosition = 0;           // Current position index (0-5)
    private int targetPosition = 0;            // Target position index (0-5)
    private boolean isHomed = false;           // Has the spindexer been homed?
    private boolean isMoving = false;          // Is the spindexer currently moving?

    // Virtual position tracking (in degrees, 0-360)
    private double virtualPositionDegrees = 0.0;
    private double targetDegrees = 0.0;

    // Ball color tracking
    private BallColor[] ballColors = new BallColor[3];  // Color of ball in each slot
    private boolean[] ballsLoaded = new boolean[3];     // Whether slot has a ball

    // Last detected color (for intake)
    private BallColor lastDetectedColor = BallColor.UNKNOWN;
    private int lastDetectedRed = 0;
    private int lastDetectedGreen = 0;
    private int lastDetectedBlue = 0;

    // Movement control
    private double currentPower = 0;           // Current servo power (-1 to 1)
    private double moveDurationMs = 0;         // How long to move for current command
    private int moveDirection = 0;             // 1 = forward, -1 = backward, 0 = stopped

    // Shooter offset state (for mechanical clearance)
    private boolean isInOffsetPosition = false;
    private double basePositionDegrees = 0;    // Position before offset was applied

    // Timers
    private ElapsedTime moveTimer = new ElapsedTime();
    private ElapsedTime settleTimer = new ElapsedTime();
    private boolean isSettling = false;

    // Homing state
    private boolean isHoming = false;

    // Default command
    private Command defaultCommand = new NullCommand();

    @Override
    public void initialize() {
        // Initialize 3 continuous rotation servos
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

        // Stop servos initially
        setServoPower(0);

        // Initialize limit switch (magnetic) - REQUIRED for homing
        try {
            this.limitSwitch = ActiveOpMode.hardwareMap()
                    .get(DigitalChannel.class, SpindexerConstants.LIMIT_SWITCH_NAME);
            limitSwitch.setMode(DigitalChannel.Mode.INPUT);
        } catch (Exception e) {
            this.limitSwitch = null;
            ActiveOpMode.telemetry().addData("WARN", "Limit switch not found - homing disabled");
        }

        // Initialize color sensors
        try {
            this.colorSensor1 = ActiveOpMode.hardwareMap()
                    .get(ColorRangeSensor.class, SpindexerConstants.COLOR_SENSOR_1_NAME);
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

    // ===== CONTINUOUS ROTATION SERVO CONTROL =====

    /**
     * Set power to all servos (-1.0 to 1.0)
     * Maps to servo values: -1 -> 0.0, 0 -> 0.5, 1 -> 1.0
     */
    private void setServoPower(double power) {
        power = Math.max(-1, Math.min(1, power));  // Clamp to -1 to 1
        this.currentPower = power;

        // Convert power (-1 to 1) to servo position (0 to 1) where 0.5 = stopped
        double servoValue = SpindexerConstants.CR_SERVO_STOP + (power * 0.5);

        if (servo1 != null) servo1.setPosition(servoValue);
        if (servo2 != null) servo2.setPosition(servoValue);
        if (servo3 != null) servo3.setPosition(servoValue);
    }

    /**
     * Get the current commanded power
     */
    public double getCurrentPower() {
        return currentPower;
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
        // Update ball detection
        updateBallDetection();

        // Handle homing
        if (isHoming) {
            if (isLimitSwitchTriggered()) {
                // Found home position
                setServoPower(0);
                virtualPositionDegrees = 0;
                currentPosition = 0;
                targetPosition = 0;
                isHomed = true;
                isHoming = false;
                isMoving = false;
            } else if (moveTimer.milliseconds() >= SpindexerConstants.HOMING_TIMEOUT_MS) {
                // Homing timeout
                setServoPower(0);
                isHoming = false;
                isMoving = false;
            }
            // Continue spinning during homing (power already set)
        }
        // Handle time-based movement
        else if (isMoving && !isSettling) {
            // Check if we've moved for the required duration
            if (moveTimer.milliseconds() >= moveDurationMs) {
                // Stop the servos
                setServoPower(0);

                // Update virtual position
                virtualPositionDegrees = targetDegrees;
                // Normalize to 0-360
                while (virtualPositionDegrees < 0) virtualPositionDegrees += 360;
                while (virtualPositionDegrees >= 360) virtualPositionDegrees -= 360;

                // Start settling
                isSettling = true;
                settleTimer.reset();
            }
        }
        // Handle settling
        else if (isSettling) {
            if (settleTimer.milliseconds() >= SpindexerConstants.SETTLE_TIME_MS) {
                isSettling = false;
                isMoving = false;
                currentPosition = targetPosition;
            }
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
        targetDegrees = positionIndex * SpindexerConstants.DEGREES_PER_POSITION;

        // Calculate shortest rotation direction
        double forwardDistance = targetDegrees - virtualPositionDegrees;
        if (forwardDistance < 0) forwardDistance += 360;

        double backwardDistance = virtualPositionDegrees - targetDegrees;
        if (backwardDistance < 0) backwardDistance += 360;

        double distanceDegrees;
        double power;

        if (SpindexerConstants.OPTIMIZE_ROTATION_DIRECTION && backwardDistance < forwardDistance) {
            // Go backward (negative direction)
            distanceDegrees = backwardDistance;
            power = -SpindexerConstants.CR_SERVO_INDEX_POWER;
            moveDirection = -1;
        } else {
            // Go forward (positive direction)
            distanceDegrees = forwardDistance;
            power = SpindexerConstants.CR_SERVO_INDEX_POWER;
            moveDirection = 1;
        }

        // Calculate time needed based on degrees and speed
        // Time = Distance / Speed, where Speed = degrees per ms at given power
        double degreesPerMs = SpindexerConstants.DEGREES_PER_MS_FULL_POWER * Math.abs(power);
        moveDurationMs = distanceDegrees / degreesPerMs;

        // Start movement
        setServoPower(power);
        isMoving = true;
        isSettling = false;
        moveTimer.reset();
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
     * Move to shooter position for a specific ball color
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
     * Move to shooter position for the slot that matches the AprilTag pattern
     */
    public boolean goToShooterPositionForTag(int tagId) {
        int greenSlot = VisionConstants.getGreenSlotForTag(tagId);
        if (greenSlot == -1) return false;

        if (ballsLoaded[greenSlot] && ballColors[greenSlot] == BallColor.GREEN) {
            goToPosition(SpindexerConstants.getShooterPosition(greenSlot));
            return true;
        }

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
     * Moves 60 degrees forward from current position.
     */
    public void applyShooterOffset() {
        if (isInOffsetPosition) return;

        basePositionDegrees = virtualPositionDegrees;

        // Calculate offset target (60 degrees forward)
        double offsetDegrees = SpindexerConstants.SHOOTER_CLEARANCE_OFFSET_DEGREES;
        targetDegrees = virtualPositionDegrees + offsetDegrees;
        if (targetDegrees >= 360) targetDegrees -= 360;

        // Calculate move duration
        double degreesPerMs = SpindexerConstants.DEGREES_PER_MS_FULL_POWER * SpindexerConstants.CR_SERVO_INDEX_POWER;
        moveDurationMs = offsetDegrees / degreesPerMs;

        // Start movement forward
        setServoPower(SpindexerConstants.CR_SERVO_INDEX_POWER);
        isMoving = true;
        isSettling = false;
        isInOffsetPosition = true;
        moveDirection = 1;
        moveTimer.reset();
    }

    /**
     * Remove offset to bring ball back to shooter position for firing.
     */
    public void removeShooterOffset() {
        if (!isInOffsetPosition) return;

        targetDegrees = basePositionDegrees;

        // Calculate offset distance (going back 60 degrees)
        double offsetDegrees = SpindexerConstants.SHOOTER_CLEARANCE_OFFSET_DEGREES;
        double degreesPerMs = SpindexerConstants.DEGREES_PER_MS_FULL_POWER * SpindexerConstants.CR_SERVO_INDEX_POWER;
        moveDurationMs = offsetDegrees / degreesPerMs;

        // Start movement backward
        setServoPower(-SpindexerConstants.CR_SERVO_INDEX_POWER);
        isMoving = true;
        isSettling = false;
        isInOffsetPosition = false;
        moveDirection = -1;
        moveTimer.reset();
    }

    /**
     * Check if spindexer is in offset position
     */
    public boolean isInOffsetPosition() {
        return isInOffsetPosition;
    }

    /**
     * Reset offset state
     */
    public void resetOffsetState() {
        isInOffsetPosition = false;
        basePositionDegrees = 0;
    }

    // ===== HOMING =====

    /**
     * Start homing routine - spins until limit switch triggers
     */
    public void startHoming() {
        if (limitSwitch == null) {
            // No limit switch - assume position 0
            virtualPositionDegrees = 0;
            currentPosition = 0;
            targetPosition = 0;
            isHomed = true;
            return;
        }

        isHomed = false;
        isHoming = true;
        isMoving = true;
        moveTimer.reset();

        // Spin slowly in reverse direction until limit switch triggers
        setServoPower(-SpindexerConstants.HOMING_POWER);
    }

    /**
     * Check if limit switch is triggered
     */
    private boolean isLimitSwitchTriggered() {
        if (limitSwitch == null) return false;

        boolean state = limitSwitch.getState();
        if (SpindexerConstants.LIMIT_SWITCH_ACTIVE_LOW) {
            return !state;  // Active-low: triggered when LOW
        } else {
            return state;   // Active-high: triggered when HIGH
        }
    }

    /**
     * Check if at home position (for external use)
     */
    public boolean isAtHome() {
        if (limitSwitch == null) {
            return currentPosition == 0 && !isMoving;
        }
        return isLimitSwitchTriggered();
    }

    /**
     * Get raw limit switch state for debugging
     */
    public boolean getLimitSwitchRawState() {
        if (limitSwitch == null) return false;
        return limitSwitch.getState();
    }

    /**
     * Complete homing routine (called when limit switch detected)
     */
    public void finishHoming() {
        setServoPower(0);
        virtualPositionDegrees = 0;
        currentPosition = 0;
        targetPosition = 0;
        isHomed = true;
        isHoming = false;
        isMoving = false;
    }

    // ===== BALL DETECTION AND COLOR =====

    private void updateBallDetection() {
        if (colorSensor1 == null) return;
        if (!isAtIntakePosition()) return;

        int slot = currentPosition / 2;

        double distance = colorSensor1.getDistance(DistanceUnit.MM);
        boolean ballDetected = distance < SpindexerConstants.COLOR_PROXIMITY_THRESHOLD;

        if (ballDetected && !ballsLoaded[slot]) {
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
        }
    }

    public boolean forceCheckBall() {
        if (colorSensor1 == null || !isAtIntakePosition()) return false;

        int slot = currentPosition / 2;
        if (ballsLoaded[slot]) return false;

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

    public void setBallColor(int slot, BallColor color) {
        if (slot >= 0 && slot < SpindexerConstants.SLOTS_COUNT) {
            ballColors[slot] = color;
            ballsLoaded[slot] = (color != BallColor.UNKNOWN);
        }
    }

    public void setBallLoaded(int slot, boolean loaded, BallColor color) {
        if (slot >= 0 && slot < SpindexerConstants.SLOTS_COUNT) {
            ballsLoaded[slot] = loaded;
            ballColors[slot] = loaded ? color : BallColor.UNKNOWN;
        }
    }

    public void setBallLoaded(int slot, boolean loaded) {
        setBallLoaded(slot, loaded, BallColor.UNKNOWN);
    }

    public void markCurrentSlotEmpty() {
        if (isAtShooterPosition()) {
            int slot = (currentPosition - 1) / 2;
            ballsLoaded[slot] = false;
            ballColors[slot] = BallColor.UNKNOWN;
        }
    }

    // ===== STATE QUERIES =====

    public boolean atPosition() {
        return !isMoving && !isSettling && !isHoming;
    }

    public boolean isAtIntakePosition() {
        return currentPosition % 2 == 0;
    }

    public boolean isAtShooterPosition() {
        return currentPosition % 2 == 1;
    }

    public boolean hasBall(int slot) {
        if (slot < 0 || slot >= SpindexerConstants.SLOTS_COUNT) return false;
        return ballsLoaded[slot];
    }

    public BallColor getBallColor(int slot) {
        if (slot < 0 || slot >= SpindexerConstants.SLOTS_COUNT) return BallColor.UNKNOWN;
        return ballColors[slot];
    }

    public BallColor[] getAllBallColors() {
        return ballColors.clone();
    }

    public boolean hasBallOfColor(BallColor color) {
        for (int i = 0; i < SpindexerConstants.SLOTS_COUNT; i++) {
            if (ballsLoaded[i] && ballColors[i] == color) {
                return true;
            }
        }
        return false;
    }

    public int countBallsOfColor(BallColor color) {
        int count = 0;
        for (int i = 0; i < SpindexerConstants.SLOTS_COUNT; i++) {
            if (ballsLoaded[i] && ballColors[i] == color) {
                count++;
            }
        }
        return count;
    }

    public int getBallCount() {
        int count = 0;
        for (boolean loaded : ballsLoaded) {
            if (loaded) count++;
        }
        return count;
    }

    public boolean isFull() {
        return getBallCount() >= SpindexerConstants.SLOTS_COUNT;
    }

    public boolean isEmpty() {
        return getBallCount() == 0;
    }

    public boolean isHomed() {
        return isHomed;
    }

    public int getCurrentPosition() {
        return currentPosition;
    }

    /**
     * Get current virtual ticks (for compatibility with motor mode telemetry)
     */
    public double getCurrentTicks() {
        return virtualPositionDegrees * SpindexerConstants.VIRTUAL_TICKS_PER_DEGREE;
    }

    /**
     * Get current virtual position in degrees
     */
    public double getVirtualPositionDegrees() {
        return virtualPositionDegrees;
    }

    public BallColor getLastDetectedColor() {
        return lastDetectedColor;
    }

    // ===== LOW-LEVEL CONTROL =====

    /**
     * Stop the spindexer
     */
    public void stop() {
        setServoPower(0);
        isMoving = false;
        isSettling = false;
        isHoming = false;
        moveDirection = 0;
    }

    /**
     * Manual spin (for testing or clearing jams)
     */
    public void spin(double power) {
        if (Math.abs(power) > 0.1) {
            setServoPower(power);
            // Update virtual position estimate based on time
            // This is approximate since we don't have feedback
        } else {
            setServoPower(0);
        }
    }

    // ===== TELEMETRY =====

    private void updateTelemetry() {
        try {
            ActiveOpMode.telemetry().addData("--- SPINDEXER (CR Servo) ---", "");
            ActiveOpMode.telemetry().addData("Homed", isHomed ? "YES" : "NO");
            ActiveOpMode.telemetry().addData("Position", "%d (%s)",
                    currentPosition, isAtIntakePosition() ? "INTAKE" : "SHOOTER");
            ActiveOpMode.telemetry().addData("Target Position", targetPosition);
            ActiveOpMode.telemetry().addData("Virtual Degrees", "%.1f°", virtualPositionDegrees);
            ActiveOpMode.telemetry().addData("Servo Power", "%.2f", currentPower);
            ActiveOpMode.telemetry().addData("At Position", atPosition() ? "YES" : "NO");
            ActiveOpMode.telemetry().addData("Is Moving", isMoving ? "YES" : "NO");

            if (limitSwitch != null) {
                boolean rawState = limitSwitch.getState();
                ActiveOpMode.telemetry().addData("Limit Switch Raw", rawState ? "HIGH" : "LOW");
                ActiveOpMode.telemetry().addData("Limit Switch", isLimitSwitchTriggered() ? "TRIGGERED" : "Open");
            } else {
                ActiveOpMode.telemetry().addData("Limit Switch", "NOT FOUND");
            }

            // Ball status with colors
            String slot0 = ballsLoaded[0] ? colorToSymbol(ballColors[0]) : "○";
            String slot1 = ballsLoaded[1] ? colorToSymbol(ballColors[1]) : "○";
            String slot2 = ballsLoaded[2] ? colorToSymbol(ballColors[2]) : "○";
            ActiveOpMode.telemetry().addData("Balls", "[%s %s %s] = %d",
                    slot0, slot1, slot2, getBallCount());

            ActiveOpMode.telemetry().addData("Green/Purple",
                    "%d G, %d P",
                    countBallsOfColor(BallColor.GREEN),
                    countBallsOfColor(BallColor.PURPLE));

            ActiveOpMode.telemetry().addData("Last Color", "%s (R:%d G:%d B:%d)",
                    lastDetectedColor.toString(),
                    lastDetectedRed, lastDetectedGreen, lastDetectedBlue);

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
