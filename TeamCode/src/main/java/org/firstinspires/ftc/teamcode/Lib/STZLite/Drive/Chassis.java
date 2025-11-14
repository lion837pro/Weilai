package org.firstinspires.ftc.teamcode.Lib.STZLite.Drive;

import static com.pedropathing.math.MathFunctions.findNormalizingScaling;

import com.pedropathing.Drivetrain;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drive.ChassisConstants;

import java.util.Arrays;
import java.util.List;

import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.MotorEx;

public class Chassis extends Drivetrain{

    public  static  final  Chassis INSTANCE = new Chassis();

    private List<MotorEx> motors;

    private VoltageSensor voltageSensor;
    private double motorCachingThreshold;
    private boolean useBrakeModeInTeleOp;
    private double staticFrictionCoefficient;

    private  MotorEx fl;
    private  MotorEx fr;
    private  MotorEx bl;
    private  MotorEx br;

    public Chassis(){
        initialize();
    }

    public void initialize(){

        HardwareMap map = ActiveOpMode.hardwareMap();

        this.maxPowerScaling = ChassisConstants.driveConstants.maxPower;
        this.motorCachingThreshold = ChassisConstants.driveConstants.motorCachingThreshold;
        this.useBrakeModeInTeleOp = ChassisConstants.driveConstants.useBrakeModeInTeleOp;

        voltageSensor = map.voltageSensor.iterator().next();
        fl = new MotorEx(ChassisConstants.flName).brakeMode();
        fr = new MotorEx(ChassisConstants.frName).brakeMode();
        bl = new MotorEx(ChassisConstants.blName).brakeMode();
        br = new MotorEx(ChassisConstants.brName).brakeMode();

        motors = Arrays.asList(fl, fr, bl, br);

        Vector copiedFrontLeftVector = ChassisConstants.driveConstants.frontLeftVector.normalize();
        vectors = new Vector[]{
                new Vector(copiedFrontLeftVector.getMagnitude(), copiedFrontLeftVector.getTheta()),
                new Vector(copiedFrontLeftVector.getMagnitude(), 2 * Math.PI - copiedFrontLeftVector.getTheta()),
                new Vector(copiedFrontLeftVector.getMagnitude(), 2 * Math.PI - copiedFrontLeftVector.getTheta()),
                new Vector(copiedFrontLeftVector.getMagnitude(), copiedFrontLeftVector.getTheta())};

        for (MotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotor().getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.getMotor().setMotorType(motorConfigurationType);
        }

        setMotorsToFloat();
        breakFollowing();
    }

    public void updateConstants() {
        fl.getMotor().setDirection(ChassisConstants.driveConstants.leftFrontMotorDirection);
        fr.getMotor().setDirection(ChassisConstants.driveConstants.rightFrontMotorDirection);
        bl.getMotor().setDirection(ChassisConstants.driveConstants.leftRearMotorDirection);
        br.getMotor().setDirection(ChassisConstants.driveConstants.rightRearMotorDirection);
        this.motorCachingThreshold = ChassisConstants.driveConstants.motorCachingThreshold;
        this.useBrakeModeInTeleOp = ChassisConstants.driveConstants.useBrakeModeInTeleOp;
        this.voltageCompensation = ChassisConstants.driveConstants.useVoltageCompensation;
        this.nominalVoltage = ChassisConstants.driveConstants.nominalVoltage;
        this.staticFrictionCoefficient = ChassisConstants.driveConstants.staticFrictionCoefficient;
    }

    /**
     * This takes in vectors for corrective power, heading power, and pathing power and outputs
     * an Array of four doubles, one for each wheel's motor power.
     * <p>
     * IMPORTANT NOTE: all vector inputs are clamped between 0 and 1 inclusive in magnitude.
     *
     * @param correctivePower this Vector includes the centrifugal force scaling Vector as well as a
     *                        translational power Vector to correct onto the Bezier curve the Follower
     *                        is following.
     * @param headingPower    this Vector points in the direction of the robot's current heading, and
     *                        the magnitude tells the robot how much it should turn and in which
     *                        direction.
     * @param pathingPower    this Vector points in the direction the robot needs to go to continue along
     *                        the Path.
     * @param robotHeading    this is the current heading of the robot, which is used to calculate how
     *                        much power to allocate to each wheel.
     * @return this returns an Array of doubles with a length of 4, which contains the wheel powers.
     */
    public double[] calculateDrive(Vector correctivePower, Vector headingPower, Vector pathingPower, double robotHeading) {
        // clamps down the magnitudes of the input vectors
        if (correctivePower.getMagnitude() > maxPowerScaling)
            correctivePower.setMagnitude(maxPowerScaling);
        if (headingPower.getMagnitude() > maxPowerScaling)
            headingPower.setMagnitude(maxPowerScaling);
        if (pathingPower.getMagnitude() > maxPowerScaling)
            pathingPower.setMagnitude(maxPowerScaling);

        // the powers for the wheel vectors
        double[] wheelPowers = new double[4];

        // This contains a copy of the mecanum wheel vectors
        Vector[] mecanumVectorsCopy = new Vector[4];

        // this contains the pathing vectors, one for each side (heading control requires 2)
        Vector[] truePathingVectors = new Vector[2];

        if (correctivePower.getMagnitude() == maxPowerScaling) {
            // checks for corrective power equal to max power scaling in magnitude. if equal, then set pathing power to that
            truePathingVectors[0] = correctivePower.copy();
            truePathingVectors[1] = correctivePower.copy();
        } else {
            // corrective power did not take up all the power, so add on heading power
            Vector leftSideVector = correctivePower.minus(headingPower);
            Vector rightSideVector = correctivePower.plus(headingPower);

            if (leftSideVector.getMagnitude() > maxPowerScaling || rightSideVector.getMagnitude() > maxPowerScaling) {
                //if the combined corrective and heading power is greater than 1, then scale down heading power
                double headingScalingFactor = Math.min(findNormalizingScaling(correctivePower, headingPower, maxPowerScaling), findNormalizingScaling(correctivePower, headingPower.times(-1), maxPowerScaling));
                truePathingVectors[0] = correctivePower.minus(headingPower.times(headingScalingFactor));
                truePathingVectors[1] = correctivePower.plus(headingPower.times(headingScalingFactor));
            } else {
                // if we're here then we can add on some drive power but scaled down to 1
                Vector leftSideVectorWithPathing = leftSideVector.plus(pathingPower);
                Vector rightSideVectorWithPathing = rightSideVector.plus(pathingPower);

                if (leftSideVectorWithPathing.getMagnitude() > maxPowerScaling || rightSideVectorWithPathing.getMagnitude() > maxPowerScaling) {
                    // too much power now, so we scale down the pathing vector
                    double pathingScalingFactor = Math.min(findNormalizingScaling(leftSideVector, pathingPower, maxPowerScaling), findNormalizingScaling(rightSideVector, pathingPower, maxPowerScaling));
                    truePathingVectors[0] = leftSideVector.plus(pathingPower.times(pathingScalingFactor));
                    truePathingVectors[1] = rightSideVector.plus(pathingPower.times(pathingScalingFactor));
                } else {
                    // just add the vectors together and you get the final vector
                    truePathingVectors[0] = leftSideVectorWithPathing.copy();
                    truePathingVectors[1] = rightSideVectorWithPathing.copy();
                }
            }
        }

        truePathingVectors[0] = truePathingVectors[0].times(2.0);
        truePathingVectors[1] = truePathingVectors[1].times(2.0);

        for (int i = 0; i < mecanumVectorsCopy.length; i++) {
            // this copies the vectors from mecanumVectors but creates new references for them
            mecanumVectorsCopy[i] = vectors[i].copy();

            mecanumVectorsCopy[i].rotateVector(robotHeading);
        }

        wheelPowers[0] = (mecanumVectorsCopy[1].getXComponent() * truePathingVectors[0].getYComponent() - truePathingVectors[0].getXComponent() * mecanumVectorsCopy[1].getYComponent()) / (mecanumVectorsCopy[1].getXComponent() * mecanumVectorsCopy[0].getYComponent() - mecanumVectorsCopy[0].getXComponent() * mecanumVectorsCopy[1].getYComponent());
        wheelPowers[1] = (mecanumVectorsCopy[0].getXComponent() * truePathingVectors[0].getYComponent() - truePathingVectors[0].getXComponent() * mecanumVectorsCopy[0].getYComponent()) / (mecanumVectorsCopy[0].getXComponent() * mecanumVectorsCopy[1].getYComponent() - mecanumVectorsCopy[1].getXComponent() * mecanumVectorsCopy[0].getYComponent());
        wheelPowers[2] = (mecanumVectorsCopy[3].getXComponent() * truePathingVectors[1].getYComponent() - truePathingVectors[1].getXComponent() * mecanumVectorsCopy[3].getYComponent()) / (mecanumVectorsCopy[3].getXComponent() * mecanumVectorsCopy[2].getYComponent() - mecanumVectorsCopy[2].getXComponent() * mecanumVectorsCopy[3].getYComponent());
        wheelPowers[3] = (mecanumVectorsCopy[2].getXComponent() * truePathingVectors[1].getYComponent() - truePathingVectors[1].getXComponent() * mecanumVectorsCopy[2].getYComponent()) / (mecanumVectorsCopy[2].getXComponent() * mecanumVectorsCopy[3].getYComponent() - mecanumVectorsCopy[3].getXComponent() * mecanumVectorsCopy[2].getYComponent());

        if (voltageCompensation) {
            double voltageNormalized = getVoltageNormalized();
            for (int i = 0; i < wheelPowers.length; i++) {
                wheelPowers[i] *= voltageNormalized;
            }
        }

        double wheelPowerMax = Math.max(Math.max(Math.abs(wheelPowers[0]), Math.abs(wheelPowers[1])), Math.max(Math.abs(wheelPowers[2]), Math.abs(wheelPowers[3])));

        if (wheelPowerMax > maxPowerScaling) {
            wheelPowers[0] = (wheelPowers[0] / wheelPowerMax) * maxPowerScaling;
            wheelPowers[1] = (wheelPowers[1] / wheelPowerMax) * maxPowerScaling;
            wheelPowers[2] = (wheelPowers[2] / wheelPowerMax) * maxPowerScaling;
            wheelPowers[3] = (wheelPowers[3] / wheelPowerMax) * maxPowerScaling;
        }

        return wheelPowers;
    }

    /**
     * This sets the motors to the zero power behavior of brake.
     */
    private void setMotorsToBrake() {
        for (MotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    /**
     * This sets the motors to the zero power behavior of float.
     */
    private void setMotorsToFloat() {
        for (MotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }

    public void breakFollowing() {
        for (MotorEx motor : motors) {
            motor.setPower(0);
        }
        setMotorsToFloat();
    }

    public void runDrive(double[] drivePowers) {
        for (int i = 0; i < motors.size(); i++) {
            if (Math.abs(motors.get(i).getPower() - drivePowers[i]) > motorCachingThreshold) {
                motors.get(i).setPower(drivePowers[i]);
            }
        }
    }

    @Override
    public void startTeleopDrive() {
        if (useBrakeModeInTeleOp) {
            setMotorsToBrake();
        }
    }

    @Override
    public void startTeleopDrive(boolean brakeMode) {
        if (brakeMode) {
            setMotorsToBrake();
        } else {
            setMotorsToFloat();
        }
    }

    public void getAndRunDrivePowers(Vector correctivePower, Vector headingPower, Vector pathingPower, double robotHeading) {
        runDrive(calculateDrive(correctivePower, headingPower, pathingPower, robotHeading));
    }

    public double xVelocity() {
        return ChassisConstants.driveConstants.xVelocity;
    }

    public double yVelocity() {
        return ChassisConstants.driveConstants.yVelocity;
    }

    public void setXVelocity(double xMovement) { ChassisConstants.driveConstants.setXVelocity(xMovement); }
    public void setYVelocity(double yMovement) { ChassisConstants.driveConstants.setYVelocity(yMovement); }

    public double getStaticFrictionCoefficient() {
        return staticFrictionCoefficient;
    }

    @Override
    public double getVoltage() {
        return voltageSensor.getVoltage();
    }

    @Override
    public String debugString() {
        return "Mecanum{" +
                " leftFront=" + fl +
                ", leftRear=" + bl +
                ", rightFront=" + fr +
                ", rightRear=" + br +
                ", motors=" + motors +
                ", motorCachingThreshold=" + motorCachingThreshold +
                ", useBrakeModeInTeleOp=" + useBrakeModeInTeleOp +
                '}';
    }

    private double getVoltageNormalized() {
        double voltage = getVoltage();
        return (nominalVoltage - (nominalVoltage * staticFrictionCoefficient)) / (voltage - ((nominalVoltage * nominalVoltage / voltage) * staticFrictionCoefficient));
    }

    public List<MotorEx> getMotors() {
        return motors;
    }

    public void stop(){
        for (MotorEx motor : motors) {
            motor.setPower(0);
        }
    }




}