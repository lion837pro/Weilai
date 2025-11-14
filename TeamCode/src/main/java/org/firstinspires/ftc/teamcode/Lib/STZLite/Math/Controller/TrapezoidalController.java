package org.firstinspires.ftc.teamcode.Lib.STZLite.Math.Controller;


import org.firstinspires.ftc.teamcode.Lib.STZLite.Math.Controller.Gains.ProfileGains;
import org.firstinspires.ftc.teamcode.Lib.STZLite.Math.Controller.Helpers.Output;
import org.firstinspires.ftc.teamcode.Lib.STZLite.Math.Controller.Helpers.TrapezoidProfile;
import org.firstinspires.ftc.teamcode.Lib.STZLite.Math.Utils.MathUtil;

/**
 * Implements a PID control loop whose setpoint is constrained by a trapezoid profile.
 */
public class TrapezoidalController{

    private final PIDController m_controller;
    private double m_minimumInput;
    private double m_maximumInput;

    private TrapezoidProfile.Constraints m_constraints;
    private TrapezoidProfile m_profile;
    private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
    private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();


    /**
     * Allocates a ProfiledPIDController with the given constants for Kp, Ki, and Kd.
     */
    public TrapezoidalController(
            double Kp, double Ki, double Kd, TrapezoidProfile.Constraints constraints) {
        m_controller = new PIDController(Kp, Ki, Kd);
        m_constraints = constraints;
        m_profile = new TrapezoidProfile(m_constraints);

    }

    public TrapezoidalController(ProfileGains.TrapezoidalGains gains){
        m_controller = new PIDController(gains.kP(), gains.kI(), gains.kD());
        m_constraints = new TrapezoidProfile.Constraints(gains.maxVelocity(), gains.maxAcceleration());
        m_profile = new TrapezoidProfile(m_constraints);
    }

    /**
     * Sets the PID Controller gain parameters.
     */
    public void setPID(double Kp, double Ki, double Kd) {
        m_controller.setPID(Kp, Ki, Kd);
    }

    /**
     * Sets the PID Controller gain parameters.
     */
    public  void setPIDGains(ProfileGains.TrapezoidalGains gains) {
        m_controller.setPID(gains.kP(), gains.kI(), gains.kD());
    }

    /**
     * Sets the PID Controller gain parameters.
     */
    public  void setPIDGains(ProfileGains.PIDGains gains){
        m_controller.setGains(gains);
    }

    /**
     * Sets the proportional coefficient of the PID controller gain.
     */
    public void setP(double Kp) {
        m_controller.setP(Kp);
    }

    /**
     * Sets the integral coefficient of the PID controller gain.
     */
    public void setI(double Ki) {
        m_controller.setI(Ki);
    }

    /**
     * Sets the differential coefficient of the PID controller gain.
     */
    public void setD(double Kd) {
        m_controller.setD(Kd);
    }

    /**
     * Sets the IZone range.
     */
    public void setIZone(double iZone) {
        m_controller.setIZone(iZone);
    }

    /**
     * Gets the proportional coefficient.
     */
    public double getP() {
        return m_controller.getP();
    }

    /**
     * Gets the integral coefficient.
     */
    public double getI() {
        return m_controller.getI();
    }

    /**
     * Gets the differential coefficient.
     */
    public double getD() {
        return m_controller.getD();
    }

    /**
     * Get the IZone range.
     */
    public double getIZone() {
        return m_controller.getIZone();
    }

    /**
     * Returns the position tolerance of this controller.
     */
    public double getPositionTolerance() {
        return m_controller.getErrorTolerance();
    }

    /**
     * Returns the velocity tolerance of this controller.
     */
    public double getVelocityTolerance() {
        return m_controller.getErrorDerivativeTolerance();
    }

    /**
     * Returns the accumulated error used in the integral calculation of this controller.
     */
    public double getAccumulatedError() {
        return m_controller.getAccumulatedError();
    }

    /**
     * Sets the goal for the ProfiledPIDController.
     */
    public void setGoal(TrapezoidProfile.State goal) {
        m_goal = goal;
    }

    /**
     * Sets the goal for the ProfiledPIDController.
     */
    public void setGoal(double goal) {
        m_goal = new TrapezoidProfile.State(goal, 0);
    }

    /**
     * Gets the goal for the ProfiledPIDController.
     */
    public TrapezoidProfile.State getGoal() {
        return m_goal;
    }

    /**
     * Returns true if the error is within the tolerance of the error.
     */
    public boolean atGoal() {
        return atSetpoint() && m_goal.equals(m_setpoint);
    }

    /**
     * Set velocity and acceleration constraints for goal.
     */
    public void setConstraints(TrapezoidProfile.Constraints constraints) {
        m_constraints = constraints;
        m_profile = new TrapezoidProfile(m_constraints);
    }

    /**
     * Get the velocity and acceleration constraints for this controller.
     */
    public TrapezoidProfile.Constraints getConstraints() {
        return m_constraints;
    }

    /**
     * Returns the current setpoint of the ProfiledPIDController.
     */
    public TrapezoidProfile.State getSetpoint() {
        return m_setpoint;
    }

    /**
     * Returns true if the error is within the tolerance of the error.
     */
    public boolean atSetpoint() {
        return m_controller.atSetpoint();
    }

    /**
     * Enables continuous input.
     */
    public void enableContinuousInput(double minimumInput, double maximumInput) {
        m_controller.enableContinuousInput(minimumInput, maximumInput);
        m_minimumInput = minimumInput;
        m_maximumInput = maximumInput;
    }

    /** Disables continuous input. */
    public void disableContinuousInput() {
        m_controller.disableContinuousInput();
    }

    /**
     * Sets the minimum and maximum contributions of the integral term.
     */
    public void setIntegratorRange(double minimumIntegral, double maximumIntegral) {
        m_controller.setIntegratorRange(minimumIntegral, maximumIntegral);
    }

    /**
     * Sets the error which is considered tolerable for use with atSetpoint().
     */
    public void setTolerance(double positionTolerance) {
        setTolerance(positionTolerance, Double.POSITIVE_INFINITY);
    }

    /**
     * Sets the error which is considered tolerable for use with atSetpoint().
     */
    public void setTolerance(double positionTolerance, double velocityTolerance) {
        m_controller.setTolerance(positionTolerance, velocityTolerance);
    }

    /**
     * Returns the difference between the setpoint and the measurement.
     */
    public double getPositionError() {
        return m_controller.getError();
    }

    /**
     * Returns the change in error per second.
     */
    public double getVelocityError() {
        return m_controller.getErrorDerivative();
    }

    /**
     * Returns the next output of the PID controller.
     */
    public Output calculate(double measurement, double dt) {
        if (m_controller.isContinuousInputEnabled()) {
            double errorBound = (m_maximumInput - m_minimumInput) / 2.0;
            double goalMinDistance =
                    MathUtil.inputModulus(m_goal.position - measurement, -errorBound, errorBound);
            double setpointMinDistance =
                    MathUtil.inputModulus(m_setpoint.position - measurement, -errorBound, errorBound);

            m_goal.position = goalMinDistance + measurement;
            m_setpoint.position = setpointMinDistance + measurement;
        }

        // --- CAMBIO AQUÍ ---
        // Pasa el 'dt' real al generador de perfiles
        // Asume que el primer argumento de 'm_profile.calculate' es el dt
        m_setpoint = m_profile.calculate(dt, m_setpoint, m_goal);

        // --- CAMBIO AQUÍ ---
        // Pasa el 'dt' real al controlador PID
        return m_controller.calculate(measurement, m_setpoint.position, dt);
    }

    /**
     * Returns the next output of the PID controller.
     */
    public Output calculate(double measurement, TrapezoidProfile.State goal, double dt) {
        setGoal(goal);
        return calculate(measurement, dt);
    }

    /**
     * Returns the next output of the PIDController.
     */
    public Output calculate(double measurement, double goal, double dt) {
        setGoal(goal);
        return calculate(measurement, dt);
    }

    /**
     * Returns the next output of the PID controller.
     */
    public Output calculate(
            double measurement, TrapezoidProfile.State goal, TrapezoidProfile.Constraints constraints, double dt) {
        setConstraints(constraints);
        return calculate(measurement, goal, dt);
    }

    /**
     * Reset the previous error and the integral term.
     */
    public void reset(TrapezoidProfile.State measurement) {
        m_controller.reset();
        m_setpoint = measurement;
    }

    /**
     * Reset the previous error and the integral term.
     */
    public void reset(double measuredPosition, double measuredVelocity) {
        reset(new TrapezoidProfile.State(measuredPosition, measuredVelocity));
    }

    /**
     * Reset the previous error and the integral term.
     */
    public void reset(double measuredPosition) {
        reset(measuredPosition, 0.0);
    }

}
