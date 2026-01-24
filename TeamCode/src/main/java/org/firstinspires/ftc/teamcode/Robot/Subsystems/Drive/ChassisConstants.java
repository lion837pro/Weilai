package org.firstinspires.ftc.teamcode.Robot.Subsystems.Drive;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ChassisConstants {

    public static final String frName = "fr";
    public static final String flName = "fl";
    public static final String brName = "br";
    public static final String blName = "bl";

    public  static  final  boolean frInverted = false;
    public  static  final  boolean flInverted = false;
    public  static  final  boolean brInverted = false;
    public  static  final  boolean blInverted = false;

    // TeleOp drive power multiplier - Tune this to match tuner speed
    // Default: 1.0 (100%), increase if TeleOp is slower than tuner
    // Typical range: 1.0 to 1.5
    public static final double TELEOP_DRIVE_POWER_SCALE = 1.3;


    // GoBilda Strafer V5 Motor Configuration
    // FL: FORWARD, FR: REVERSE, BL: REVERSE, BR: FORWARD
    public static final MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName(frName)
            .rightRearMotorName(brName)
            .leftRearMotorName(blName)
            .leftFrontMotorName(flName)
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(65.19982574)
            .yVelocity(50.48943727);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(   3.3)
            .strafePodX(-6.6)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);


    /**
     These are the PathConstraints in order:
     tValueConstraint, velocityConstraint, translationalConstraint, headingConstraint, timeoutConstraint,
     brakingStrength, BEZIER_CURVE_SEARCH_LIMIT, brakingStart
     The BEZIER_CURVE_SEARCH_LIMIT should typically be left at 10 and shouldn't be changed.

     velocityConstraint: Velocity tolerance (inches/sec) - PID engages when velocity error exceeds this
     translationalConstraint: Position tolerance (inches) - PID engages when position error exceeds this

     Previous values (0.02, 0.01) were too tight - caused small errors to not get corrected
     because secondary PIDs were too weak. Now using reasonable thresholds.
     */
    public static PathConstraints pathConstraints = new PathConstraints(
            0.995,
            0.5,      // velocityConstraint: Reasonable threshold for velocity corrections
            0.25,     // translationalConstraint: Reasonable threshold for position corrections
            0.009,
            50,
            1.25,
            10,
            1
    );

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(8.1)
            .forwardZeroPowerAcceleration(-29.73161487098345)
            .lateralZeroPowerAcceleration(-30.822693520646286)
            // Primary translational PID - used when error > translationalPIDFSwitch (4 inches)
            .translationalPIDFCoefficients(new PIDFCoefficients(
                    0.2,
                    0,
                    0.01,
                    0.015
            ))
            .translationalPIDFSwitch(4)
            // Secondary translational PID - used when error < 4 inches
            // FIXED: Previous values (0.0, 0, 0, 0.0006) were essentially disabled
            // Now using reasonable values for fine position corrections
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(
                    0.15,   // kP: Reasonable proportional gain for small errors
                    0,
                    0.005,  // kD: Small derivative for damping
                    0.01    // kF: Small feedforward
            ))
            // Primary heading PID - used when error > headingPIDFSwitch
            .headingPIDFCoefficients(new PIDFCoefficients(
                    0.8,
                    0,
                    0,
                    0.01
            ))
            // Secondary heading PID - used for fine heading corrections
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(
                    2.5,
                    0,
                    0.1,
                    0.0005
            ))
            // Primary drive PID - used when error > drivePIDFSwitch (15)
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0.1,
                    0,
                    0.00035,
                    0.6,
                    0.015
            ))
            // Secondary drive PID - used when error < 15
            // FIXED: Previous values (0.02, 0, 0.000005, 0.6, 0.01) were too weak
            // Now using reasonable values for fine drive corrections
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0.08,     // kP: Reasonable proportional gain for small errors
                    0,
                    0.0002,   // kD: Small derivative for damping
                    0.6,      // Filter coefficient
                    0.012     // kF: Small feedforward
            ))
            .drivePIDFSwitch(15)
            .centripetalScaling(0.0005);

    public static Follower buildPedroPathing (HardwareMap hardwareMap){
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .build();
    }

}
