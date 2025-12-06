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


    // GoBilda Strafer V5 Standard Motor Configuration
    // FL: REVERSE, FR: FORWARD, BL: REVERSE, BR: FORWARD
    public static final MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName(frName)
            .rightRearMotorName(brName)
            .leftRearMotorName(blName)
            .leftFrontMotorName(flName)
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
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
     The BEZIER_CURVE_SEARCH_LIMIT should typically be left at 10 and srhouldn't be changed.

     velocityConstraint: Velocity tolerance (inches/sec) - PID engages when velocity error exceeds this
     translationalConstraint: Position tolerance (inches) - PID engages when position error exceeds this
     */
    public static PathConstraints pathConstraints = new PathConstraints(
            0.995,
            0.02,     // velocityConstraint: Reduced from 0.1 to 0.02 for tighter control
            0.01,     // translationalConstraint: Ultra-tight 0.01 for maximum PID responsiveness
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
            .translationalPIDFCoefficients(new PIDFCoefficients(
                    0.2,
                    0,
                    0.01,
                    0.015
            ))
            .translationalPIDFSwitch(4)
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(
                    0.0,
                    0,
                    0.,
                    0.0006
            ))
            .headingPIDFCoefficients(new PIDFCoefficients(
                    0.8,
                    0,
                    0,
                    0.01
            ))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(
                    2.5,
                    0,
                    0.1,
                    0.0005
            ))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0.1,
                    0,
                    0.00035,
                    0.6,
                    0.015
            ))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0.02,
                    0,
                    0.000005,
                    0.6,
                    0.01
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
