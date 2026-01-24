package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Simplified Autonomous with Spindexer and Shooter
 *
 * State Machine:
 * 0: Follow path1 to shooting position
 * 1: Path complete - start shooter motor
 * 2: Wait for shooter spinup
 * 3: Move spindexer to shooter position + fire ball 1
 * 4: Fire ball 2
 * 5: Fire ball 3
 * 6: Stop all motors - done
 */
@Autonomous(name = "autoRed1", group = "auto-pedro")
public class autoRed1 extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    private Paths paths;
    private final Pose startPose = new Pose(84, 12, Math.toRadians(90));

    // ===== HARDWARE =====
    private DcMotorEx shooterMotor;    // "Sh1" - shooter flywheel
    private DcMotorEx spindexerMotor;  // "spin" - spindexer rotation
    private Servo feederServo;         // "feeder" - pushes ball to shooter

    // ===== CONSTANTS =====
    // Shooter
    private static final double SHOOTER_POWER = 0.6;  // 60% power for shooting
    private static final int SHOOTER_SPINUP_MS = 1000;  // Wait 1 second for spinup

    // Spindexer
    private static final double SPINDEXER_POWER = 0.3;  // Power for spindexer rotation
    private static final double TICKS_PER_POSITION = 537.7 / 6.0;  // 60 degrees = 1 position

    // Feeder servo
    private static final double FEEDER_DOWN = 0.0;    // Resting position
    private static final double FEEDER_UP = 0.667;    // Push ball position
    private static final int FEEDER_MOVE_MS = 250;    // Time to move servo

    // Shooting sequence
    private int ballsShot = 0;
    private int shootingSubState = 0;  // Sub-state for ball firing sequence

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
                    // Start shooter motor
                    shooterMotor.setPower(SHOOTER_POWER);
                    actionTimer.resetTimer();
                    setPathState(2);
                }
                break;

            case 2:
                // Wait for shooter to spin up
                if (actionTimer.getElapsedTimeSeconds() > (SHOOTER_SPINUP_MS / 1000.0)) {
                    // Shooter ready, start shooting sequence
                    ballsShot = 0;
                    shootingSubState = 0;
                    setPathState(3);
                }
                break;

            case 3:
                // Shooting sequence - fires all 3 balls
                shootBallSequence();
                break;

            case 4:
                // Done - stop all motors
                shooterMotor.setPower(0);
                spindexerMotor.setPower(0);
                feederServo.setPosition(FEEDER_DOWN);
                setPathState(-1);  // End autonomous
                break;
        }
    }

    /**
     * Sub-state machine for shooting balls
     * Each ball: move spindexer → feeder up → feeder down → next ball
     */
    private void shootBallSequence() {
        switch (shootingSubState) {
            case 0:
                // Move spindexer to shooter position (odd positions: 1, 3, 5)
                // For simplicity, just rotate by 1 position (60 degrees)
                spindexerMotor.setTargetPosition(spindexerMotor.getCurrentPosition() + (int)TICKS_PER_POSITION);
                spindexerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                spindexerMotor.setPower(SPINDEXER_POWER);
                shootingSubState = 1;
                break;

            case 1:
                // Wait for spindexer to reach position
                if (!spindexerMotor.isBusy()) {
                    spindexerMotor.setPower(0);
                    // Push ball with feeder
                    feederServo.setPosition(FEEDER_UP);
                    actionTimer.resetTimer();
                    shootingSubState = 2;
                }
                break;

            case 2:
                // Wait for feeder to push ball
                if (actionTimer.getElapsedTimeSeconds() > (FEEDER_MOVE_MS / 1000.0)) {
                    // Bring feeder back down
                    feederServo.setPosition(FEEDER_DOWN);
                    actionTimer.resetTimer();
                    shootingSubState = 3;
                }
                break;

            case 3:
                // Wait for feeder to return
                if (actionTimer.getElapsedTimeSeconds() > (FEEDER_MOVE_MS / 1000.0)) {
                    ballsShot++;

                    if (ballsShot >= 3) {
                        // All balls shot, done
                        setPathState(4);
                    } else {
                        // Move to next ball
                        shootingSubState = 0;
                    }
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
        telemetry.addData("Sub-State", shootingSubState);
        telemetry.addData("Balls Shot", ballsShot);
        telemetry.addData("", "");
        telemetry.addData("Pose X", "%.1f", follower.getPose().getX());
        telemetry.addData("Pose Y", "%.1f", follower.getPose().getY());
        telemetry.addData("Heading", "%.1f°", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("", "");
        telemetry.addData("Shooter Power", "%.2f", shooterMotor.getPower());
        telemetry.addData("Spindexer Pos", spindexerMotor.getCurrentPosition());
        telemetry.update();
    }

    @Override
    public void init() {
        // Initialize timers
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        // Initialize Pedro follower
        follower = Constants.createFollower(hardwareMap);
        paths = new Paths(follower);
        follower.setStartingPose(startPose);

        // Initialize shooter motor
        shooterMotor = hardwareMap.get(DcMotorEx.class, "Sh1");
        shooterMotor.setDirection(DcMotorEx.Direction.REVERSE);  // Inverted per ShooterConstants
        shooterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        shooterMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize spindexer motor
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
        telemetry.addData("Shooter", "Sh1");
        telemetry.addData("Spindexer", "spin");
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
        if (shooterMotor != null) shooterMotor.setPower(0);
        if (spindexerMotor != null) spindexerMotor.setPower(0);
        if (feederServo != null) feederServo.setPosition(FEEDER_DOWN);
    }
}
