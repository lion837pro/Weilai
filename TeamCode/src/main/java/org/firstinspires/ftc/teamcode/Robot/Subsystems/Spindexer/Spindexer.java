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
import dev.nextftc.hardware.impl.MotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drive.VisionConstants;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drive.VisionConstants.BallColor;

/**
 * Spindexer (Spinning Indexer) Subsystem
 *
 * A rotating indexer that holds 3 balls with 6 preset positions.
 * Uses a 312 RPM motor with encoder for precise position control.
 * Uses a magnetic limit switch for homing and 2 color sensors for ball detection.
 * Tracks ball colors (GREEN or PURPLE) for automatic color sorting with AprilTags.
 *
 * MOTOR-BASED CONTROL:
 * - Uses encoder feedback for precise positioning
 * - PID control for smooth movement
 * - Limit switch is used for homing to establish position 0
 *
 * Slot mapping:
 * - Slot 0: Positions 0 (intake) and 1 (shooter)
 * - Slot 1: Positions 2 (intake) and 3 (shooter)
 * - Slot 2: Positions 4 (intake) and 5 (shooter)
 */
public class Spindexer implements Subsystem {

    public static final Spindexer INSTANCE = new Spindexer();

    // Hardware - Motor
    private MotorEx motor;

    // Hardware - Sensors
    private DigitalChannel limitSwitch;
    private ColorRangeSensor colorSensor1;
    private ColorRangeSensor colorSensor2;

    // Hardware - Feeder servo (transfers ball from spindexer to shooter)
    private Servo feederServo;
    private boolean isFeederUp = false;

    // State tracking
    private int currentPosition = 0;           // Current position index (0-5)
    private int targetPosition = 0;            // Target position index (0-5)
    private boolean isHomed = false;           // Has the spindexer been homed?
    private boolean isMoving = false;          // Is the spindexer currently moving?

    // Encoder tracking
    private double encoderOffset = 0;          // Virtual encoder reset offset
    private double targetTicks = 0;            // Target encoder position

    // PID state
    private double lastError = 0;
    private long lastPIDTime = 0;

    // Ball color tracking
    private BallColor[] ballColors = new BallColor[3];
    private boolean[] ballsLoaded = new boolean[3];

    // Last detected color (for intake)
    private BallColor lastDetectedColor = BallColor.UNKNOWN;
    private int lastDetectedRed = 0;
    private int lastDetectedGreen = 0;
    private int lastDetectedBlue = 0;

    // Movement control
    private double currentPower = 0;

    // Shooter offset state (for mechanical clearance)
    private boolean isInOffsetPosition = false;
    private double basePositionTicks = 0;

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
        // Initialize motor
        try {
            motor = new MotorEx(SpindexerConstants.MOTOR_NAME);
            if (SpindexerConstants.MOTOR_INVERTED) {
                motor.reversed();
            }
            motor.brakeMode();
            motor.resetEncoder();
        } catch (Exception e) {
            motor = null;
            ActiveOpMode.telemetry().addData("Spindexer Motor", "NOT FOUND: " + e.getMessage());
        }

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
                ((SwitchableLight) this.colorSensor1).enableLight(false);
            }
            this.colorSensor1.setGain(SpindexerConstants.COLOR_SENSOR_GAIN);
        } catch (Exception e) {
            this.colorSensor1 = null;
        }

        try {
            this.colorSensor2 = ActiveOpMode.hardwareMap()
                    .get(ColorRangeSensor.class, SpindexerConstants.COLOR_SENSOR_2_NAME);
            if (this.colorSensor2 instanceof SwitchableLight) {
                ((SwitchableLight) this.colorSensor2).enableLight(false);
            }
            this.colorSensor2.setGain(SpindexerConstants.COLOR_SENSOR_GAIN);
        } catch (Exception e) {
            this.colorSensor2 = null;
        }

        // Initialize feeder servo
        try {
            this.feederServo = ActiveOpMode.hardwareMap()
                    .get(Servo.class, SpindexerConstants.FEEDER_SERVO_NAME);
            if (SpindexerConstants.FEEDER_SERVO_REVERSED) {
                feederServo.setDirection(Servo.Direction.REVERSE);
            }
            // Start in down position
            feederServo.setPosition(SpindexerConstants.FEEDER_DOWN_POSITION);
            isFeederUp = false;
        } catch (Exception e) {
            this.feederServo = null;
            ActiveOpMode.telemetry().addData("Feeder Servo", "NOT FOUND");
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
        if (motor == null) return;

        // Update ball detection
        updateBallDetection();

        // Handle homing
        if (isHoming) {
            if (isLimitSwitchTriggered()) {
                // Found home position
                motor.setPower(0);
                resetEncoder();
                currentPosition = 0;
                targetPosition = 0;
                targetTicks = 0;
                isHomed = true;
                isHoming = false;
                isMoving = false;
            } else if (moveTimer.milliseconds() >= SpindexerConstants.HOMING_TIMEOUT_MS) {
                // Homing timeout
                motor.setPower(0);
                isHoming = false;
                isMoving = false;
            }
            // Continue spinning during homing (power already set)
        }
        // Handle position control
        else if (isMoving && !isSettling) {
            runPositionPID();
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

    // ===== POSITION PID CONTROL =====

    private void runPositionPID() {
        long currentTime = System.nanoTime();
        double dt = lastPIDTime == 0 ? 0.02 : (currentTime - lastPIDTime) / 1e9;
        lastPIDTime = currentTime;

        double currentTicks = getCurrentTicks();
        double error = targetTicks - currentTicks;

        // Check if at position
        if (Math.abs(error) <= SpindexerConstants.POSITION_TOLERANCE) {
            motor.setPower(0);
            currentPower = 0;
            // Start settling
            isSettling = true;
            settleTimer.reset();
            return;
        }

        // PD control
        double p = SpindexerConstants.kP * error;
        double derivative = (error - lastError) / dt;
        double d = SpindexerConstants.kD * derivative;
        lastError = error;

        double power = p + d;

        // Add static friction compensation
        if (Math.abs(power) > 0.01) {
            power += Math.signum(power) * SpindexerConstants.kS;
        }

        // Clamp power
        power = Math.max(-SpindexerConstants.MAX_POWER,
                Math.min(SpindexerConstants.MAX_POWER, power));

        motor.setPower(power);
        currentPower = power;
    }

    // ===== ENCODER METHODS =====

    private double getCurrentTicks() {
        if (motor == null) return 0;
        return motor.getCurrentPosition() - encoderOffset;
    }

    private void resetEncoder() {
        if (motor != null) {
            encoderOffset = motor.getCurrentPosition();
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
        double targetPositionTicks = SpindexerConstants.getPositionTicks(positionIndex);
        double currentTicks = getCurrentTicks();

        // Calculate shortest rotation direction
        double ticksPerRev = SpindexerConstants.TICKS_PER_SPINDEXER_REV;
        double forwardDistance = targetPositionTicks - currentTicks;
        if (forwardDistance < 0) forwardDistance += ticksPerRev;

        double backwardDistance = currentTicks - targetPositionTicks;
        if (backwardDistance < 0) backwardDistance += ticksPerRev;

        if (SpindexerConstants.OPTIMIZE_ROTATION_DIRECTION && backwardDistance < forwardDistance) {
            // Go backward
            targetTicks = currentTicks - backwardDistance;
        } else {
            // Go forward
            targetTicks = currentTicks + forwardDistance;
        }

        // Reset PID state
        lastError = 0;
        lastPIDTime = 0;

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
     */
    public void applyShooterOffset() {
        if (isInOffsetPosition) return;

        basePositionTicks = getCurrentTicks();
        targetTicks = basePositionTicks + SpindexerConstants.SHOOTER_CLEARANCE_OFFSET_TICKS;

        lastError = 0;
        lastPIDTime = 0;

        isMoving = true;
        isSettling = false;
        isInOffsetPosition = true;
        moveTimer.reset();
    }

    /**
     * Remove offset to bring ball back to shooter position for firing.
     */
    public void removeShooterOffset() {
        if (!isInOffsetPosition) return;

        targetTicks = basePositionTicks;

        lastError = 0;
        lastPIDTime = 0;

        isMoving = true;
        isSettling = false;
        isInOffsetPosition = false;
        moveTimer.reset();
    }

    public boolean isInOffsetPosition() {
        return isInOffsetPosition;
    }

    public void resetOffsetState() {
        isInOffsetPosition = false;
        basePositionTicks = 0;
    }

    // ===== HOMING =====

    /**
     * Start homing routine - spins until limit switch triggers
     */
    public void startHoming() {
        if (limitSwitch == null) {
            // No limit switch - assume position 0
            resetEncoder();
            currentPosition = 0;
            targetPosition = 0;
            targetTicks = 0;
            isHomed = true;
            return;
        }

        isHomed = false;
        isHoming = true;
        isMoving = true;
        moveTimer.reset();

        // Spin slowly in reverse direction until limit switch triggers
        if (motor != null) {
            motor.setPower(-SpindexerConstants.HOMING_POWER);
        }
    }

    private boolean isLimitSwitchTriggered() {
        if (limitSwitch == null) return false;

        boolean state = limitSwitch.getState();
        if (SpindexerConstants.LIMIT_SWITCH_ACTIVE_LOW) {
            return !state;
        } else {
            return state;
        }
    }

    public boolean isAtHome() {
        if (limitSwitch == null) {
            return currentPosition == 0 && !isMoving;
        }
        return isLimitSwitchTriggered();
    }

    public boolean getLimitSwitchRawState() {
        if (limitSwitch == null) return false;
        return limitSwitch.getState();
    }

    public void finishHoming() {
        if (motor != null) motor.setPower(0);
        resetEncoder();
        currentPosition = 0;
        targetPosition = 0;
        targetTicks = 0;
        isHomed = true;
        isHoming = false;
        isMoving = false;
    }

    // ===== BALL DETECTION AND COLOR =====

    private boolean intakeSensorLightEnabled = false;
    private boolean shooterSensorLightEnabled = false;
    private boolean ballWasAtShooter = false;

    private void setIntakeSensorLight(boolean enabled) {
        if (intakeSensorLightEnabled == enabled) return;
        intakeSensorLightEnabled = enabled;
        if (colorSensor1 instanceof SwitchableLight) {
            ((SwitchableLight) colorSensor1).enableLight(enabled);
        }
    }

    private void setShooterSensorLight(boolean enabled) {
        if (shooterSensorLightEnabled == enabled) return;
        shooterSensorLightEnabled = enabled;
        if (colorSensor2 instanceof SwitchableLight) {
            ((SwitchableLight) colorSensor2).enableLight(enabled);
        }
    }

    private void updateBallDetection() {
        updateIntakeSensor();
        updateShooterSensor();
    }

    private void updateIntakeSensor() {
        if (colorSensor1 == null) return;

        boolean shouldEnable = isAtIntakePosition() && !isMoving;
        setIntakeSensorLight(shouldEnable);

        if (!isAtIntakePosition() || isMoving) return;

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

    private void updateShooterSensor() {
        if (colorSensor2 == null) return;

        boolean shouldEnable = isAtShooterPosition() && !isMoving;
        setShooterSensorLight(shouldEnable);

        if (!isAtShooterPosition() || isMoving) {
            ballWasAtShooter = false;
            return;
        }

        int slot = (currentPosition - 1) / 2;

        double distance = colorSensor2.getDistance(DistanceUnit.MM);
        boolean ballPresent = distance < SpindexerConstants.COLOR_PROXIMITY_THRESHOLD;

        if (ballWasAtShooter && !ballPresent && ballsLoaded[slot]) {
            ballsLoaded[slot] = false;
            ballColors[slot] = BallColor.UNKNOWN;
        }

        ballWasAtShooter = ballPresent;
    }

    public boolean isBallAtShooter() {
        if (colorSensor2 == null || !isAtShooterPosition()) return false;

        setShooterSensorLight(true);
        double distance = colorSensor2.getDistance(DistanceUnit.MM);
        return distance < SpindexerConstants.COLOR_PROXIMITY_THRESHOLD;
    }

    public boolean forceCheckBall() {
        if (colorSensor1 == null || !isAtIntakePosition()) return false;

        int slot = currentPosition / 2;
        if (ballsLoaded[slot]) return false;

        setIntakeSensorLight(true);

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

    /**
     * Mark a slot as loaded (for human player feeding balls - no color detection)
     */
    public void markSlotLoaded(int slot) {
        if (slot >= 0 && slot < SpindexerConstants.SLOTS_COUNT) {
            ballsLoaded[slot] = true;
            // No color detection - mark as UNKNOWN
            ballColors[slot] = BallColor.UNKNOWN;
        }
    }

    /**
     * Mark current intake slot as loaded (for human player feeding)
     */
    public void markCurrentIntakeSlotLoaded() {
        if (isAtIntakePosition()) {
            int slot = currentPosition / 2;
            ballsLoaded[slot] = true;
            ballColors[slot] = BallColor.UNKNOWN;
        }
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

    public double getCurrentTicks_Public() {
        return getCurrentTicks();
    }

    public BallColor getLastDetectedColor() {
        return lastDetectedColor;
    }

    public double getCurrentPower() {
        return currentPower;
    }

    // ===== LOW-LEVEL CONTROL =====

    public void stop() {
        if (motor != null) motor.setPower(0);
        currentPower = 0;
        isMoving = false;
        isSettling = false;
        isHoming = false;
    }

    public void spin(double power) {
        if (motor == null) return;

        if (Math.abs(power) > 0.1) {
            motor.setPower(power * SpindexerConstants.MAX_POWER);
            currentPower = power;
            isMoving = false;  // Manual control, not position control
        } else {
            motor.setPower(0);
            currentPower = 0;
        }
    }

    // ===== FEEDER SERVO CONTROL =====

    /**
     * Move feeder servo to UP position (120 degrees) to push ball into shooter
     */
    public void feederUp() {
        if (feederServo == null) return;
        feederServo.setPosition(SpindexerConstants.FEEDER_UP_POSITION);
        isFeederUp = true;
    }

    /**
     * Move feeder servo to DOWN position (0 degrees) - resting position
     */
    public void feederDown() {
        if (feederServo == null) return;
        feederServo.setPosition(SpindexerConstants.FEEDER_DOWN_POSITION);
        isFeederUp = false;
    }

    /**
     * Check if feeder is in UP position
     */
    public boolean isFeederUp() {
        return isFeederUp;
    }

    /**
     * Check if feeder servo is initialized
     */
    public boolean hasFeederServo() {
        return feederServo != null;
    }

    // ===== TELEMETRY =====

    private void updateTelemetry() {
        try {
            ActiveOpMode.telemetry().addData("--- SPINDEXER (Motor) ---", "");
            ActiveOpMode.telemetry().addData("Homed", isHomed ? "YES" : "NO");
            ActiveOpMode.telemetry().addData("Position", "%d (%s)",
                    currentPosition, isAtIntakePosition() ? "INTAKE" : "SHOOTER");
            ActiveOpMode.telemetry().addData("Target Position", targetPosition);
            ActiveOpMode.telemetry().addData("Encoder Ticks", "%.1f", getCurrentTicks());
            ActiveOpMode.telemetry().addData("Target Ticks", "%.1f", targetTicks);
            ActiveOpMode.telemetry().addData("Motor Power", "%.2f", currentPower);
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

            // Intake sensor (colorSensor1)
            if (colorSensor1 != null) {
                try {
                    double dist1 = colorSensor1.getDistance(DistanceUnit.MM);
                    boolean ballAtIntake = dist1 < SpindexerConstants.COLOR_PROXIMITY_THRESHOLD;
                    ActiveOpMode.telemetry().addData("INTAKE Sensor", "%.1fmm %s %s",
                            dist1,
                            ballAtIntake ? "BALL" : "empty",
                            intakeSensorLightEnabled ? "(ON)" : "(off)");
                } catch (Exception e) {
                    ActiveOpMode.telemetry().addData("INTAKE Sensor", "ERROR");
                }
            } else {
                ActiveOpMode.telemetry().addData("INTAKE Sensor", "NOT FOUND");
            }

            // Shooter sensor (colorSensor2)
            if (colorSensor2 != null) {
                try {
                    double dist2 = colorSensor2.getDistance(DistanceUnit.MM);
                    boolean ballAtShooter = dist2 < SpindexerConstants.COLOR_PROXIMITY_THRESHOLD;
                    ActiveOpMode.telemetry().addData("SHOOTER Sensor", "%.1fmm %s %s",
                            dist2,
                            ballAtShooter ? "BALL" : "empty",
                            shooterSensorLightEnabled ? "(ON)" : "(off)");
                } catch (Exception e) {
                    ActiveOpMode.telemetry().addData("SHOOTER Sensor", "ERROR");
                }
            } else {
                ActiveOpMode.telemetry().addData("SHOOTER Sensor", "NOT FOUND");
            }

            // Feeder servo status
            if (feederServo != null) {
                ActiveOpMode.telemetry().addData("Feeder", isFeederUp ? "UP (120°)" : "DOWN (0°)");
            } else {
                ActiveOpMode.telemetry().addData("Feeder", "NOT FOUND");
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
