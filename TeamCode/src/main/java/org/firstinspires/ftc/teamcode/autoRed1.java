package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Autonomous with Spindexer and Shooter
 * Uses your original shooting sequence pattern with velocity control.
 *
 * State Machine:
 * 0: Follow path1 to shooting position
 * 1: Path complete - start shooter motor (spinup)
 * 2: Wait 1 second, then start spindexer + feeder cycling
 * 3: Run spindexer + cycle feeder for 3 seconds to shoot all balls
 * 4: Stop all motors - done
 */
@Autonomous(name = "autoRed1", group = "auto-pedro")
public class autoRed1 extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer, feederTimer;

    private int pathState;

    private Paths paths;
    private final Pose startPose = new Pose(84, 12, Math.toRadians(90));

    // ===== HARDWARE =====
    private DcMotorEx shooterMotor;    // "Sh1" - shooter flywheel
    private DcMotorEx spindexerMotor;  // "spin" - spindexer rotation
    private Servo feederServo;         // "feeder" - pushes ball to shooter

    // ===== VELOCITY CONSTANTS (matching your original pattern) =====
    // Shooter velocity (was leftPelvis/rightPelvis at 1650)
    private static final int SHOOTER_VELOCITY = 1650;

    // Spindexer velocity (was inter at 1900)
    private static final int SPINDEXER_VELOCITY = 1900;

    // Feeder servo
    private static final double FEEDER_DOWN = 0.0;    // Resting position
    private static final double FEEDER_UP = 0.667;    // Push ball position

    // Timing (matching your original pattern)
    private static final double SPINUP_TIME = 1.0;       // 1 second spinup
    private static final double SHOOTING_TIME = 3.0;     // 3 seconds to shoot all balls
    private static final double FEEDER_CYCLE_TIME = 0.4; // Feeder up/down cycle time

    // Feeder cycling state
    private boolean feederUp = false;

    public static class Paths {
        public PathChain launch1;

        public Paths(Follower follower) {
            launch1 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(84.000, 12.000),
                            new Pose(100.000, 100.000)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(45))
            .build();
        }
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // Start following path to shooting position
                follower.followPath(paths.launch1);
                setPathState(1);
                break;

            case 1:
                // Wait for path to complete
                if (!follower.isBusy()) {
                    // Turn on shooter motor (like leftPelvis/rightPelvis)
                    shooterMotor.setVelocity(SHOOTER_VELOCITY);
                    actionTimer.resetTimer();
                    setPathState(2);
                }
                break;

            case 2:
                // Wait 1 second for spinup (like your original pattern)
                if (actionTimer.getElapsedTimeSeconds() > SPINUP_TIME) {
                    // Turn on spindexer (like inter)
                    spindexerMotor.setVelocity(SPINDEXER_VELOCITY);
                    // Start feeder cycling
                    feederServo.setPosition(FEEDER_UP);
                    feederUp = true;
                    feederTimer.resetTimer();
                    actionTimer.resetTimer();
                    setPathState(3);
                }
                break;

            case 3:
                // Run spindexer + cycle feeder for 3 seconds
                // Cycle feeder up/down to push balls
                if (feederTimer.getElapsedTimeSeconds() > FEEDER_CYCLE_TIME / 2) {
                    if (feederUp) {
                        feederServo.setPosition(FEEDER_DOWN);
                        feederUp = false;
                    } else {
                        feederServo.setPosition(FEEDER_UP);
                        feederUp = true;
                    }
                    feederTimer.resetTimer();
                }

                // After 3 seconds, stop everything
                if (actionTimer.getElapsedTimeSeconds() > SHOOTING_TIME) {
                    // Turn everything off (like your original pattern)
                    shooterMotor.setVelocity(0);
                    spindexerMotor.setVelocity(0);
                    feederServo.setPosition(FEEDER_DOWN);
                    setPathState(-1);  // End autonomous
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        // Telemetry
        telemetry.addData("--- AUTO RED 1 ---", "");
        telemetry.addData("State", pathState);
        telemetry.addData("Timer", "%.1f s", actionTimer.getElapsedTimeSeconds());
        telemetry.addData("", "");
        telemetry.addData("Pose X", "%.1f", follower.getPose().getX());
        telemetry.addData("Pose Y", "%.1f", follower.getPose().getY());
        telemetry.addData("Heading", "%.1f°", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("", "");
        telemetry.addData("Shooter Vel", "%.0f", shooterMotor.getVelocity());
        telemetry.addData("Spindexer Vel", "%.0f", spindexerMotor.getVelocity());
        telemetry.addData("Feeder", feederUp ? "UP" : "DOWN");
        telemetry.update();
    }

    @Override
    public void init() {
        // Initialize timers
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        feederTimer = new Timer();
        opmodeTimer.resetTimer();

        // Initialize Pedro follower
        follower = Constants.createFollower(hardwareMap);
        paths = new Paths(follower);
        follower.setStartingPose(startPose);

        // Initialize shooter motor (velocity mode)
        shooterMotor = hardwareMap.get(DcMotorEx.class, "Sh1");
        shooterMotor.setDirection(DcMotorEx.Direction.REVERSE);
        shooterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        shooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Initialize spindexer motor (velocity mode)
        spindexerMotor = hardwareMap.get(DcMotorEx.class, "spin");
        spindexerMotor.setDirection(DcMotorEx.Direction.FORWARD);
        spindexerMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        spindexerMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        spindexerMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Initialize feeder servo (reversed)
        feederServo = hardwareMap.get(Servo.class, "feeder");
        feederServo.setDirection(Servo.Direction.REVERSE);
        feederServo.setPosition(FEEDER_DOWN);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Shooter", "Sh1 (vel: " + SHOOTER_VELOCITY + ")");
        telemetry.addData("Spindexer", "spin (vel: " + SPINDEXER_VELOCITY + ")");
        telemetry.addData("Feeder", "feeder");
        telemetry.update();
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void stop() {
        // Safety - stop all motors
        if (shooterMotor != null) shooterMotor.setVelocity(0);
        if (spindexerMotor != null) spindexerMotor.setVelocity(0);
        if (feederServo != null) feederServo.setPosition(FEEDER_DOWN);
    }
}
