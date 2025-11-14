package org.firstinspires.ftc.teamcode.Lib.STZLite.Math.Controller.Gains;

public class ProfileGains {

    private ProfileGains() {
    }

    public interface Gains {

        default double[] toArray() {
            return new double[]{};
        }

        default Gains fromArray(double[] array) {
            return new Gains() {
            };
        }

    }

    public static final class PIDGains implements Gains{
        private final double kP;
        private final double kI;
        private final double kD;

        public PIDGains(double kP, double kI, double kD) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
        }

        public double kP() {
            return kP;
        }

        public double kI() {
            return kI;
        }

        public double kD() {
            return kD;
        }

        @Override
        public double[] toArray(){
            return new double[]{kP,kI,kD};
        }

        @Override
        public Gains fromArray(double[] array){
            return new PIDGains(array[0], array[1], array[2]);
        }

        @Override
        public boolean equals(Object obj) {
            if (obj == this) return true;
            if (obj == null || obj.getClass() != this.getClass()) return false;
            PIDGains that = (PIDGains) obj;
            return Double.doubleToLongBits(this.kP) == Double.doubleToLongBits(that.kP) &&
                    Double.doubleToLongBits(this.kI) == Double.doubleToLongBits(that.kI) &&
                    Double.doubleToLongBits(this.kD) == Double.doubleToLongBits(that.kD);
        }

    }

    public static class FeedForwardGains implements Gains{
        private final double kS;
        private final double kV;
        private final double kA;

        public FeedForwardGains(double kS, double kV, double kA) {
            this.kS = kS;
            this.kV = kV;
            this.kA = kA;
        }

        public double kS() {
            return kS;
        }

        public double kV() {
            return kV;
        }

        public double kA() {
            return kA;
        }

        @Override
        public double[] toArray(){
            return new double[]{kS, kV, kA};
        }

        @Override
        public Gains fromArray(double[]array){
            return new FeedForwardGains(array[0], array[1], array[2]);
        }

        @Override
        public boolean equals(Object obj) {
            if (obj == this) return true;
            if (obj == null || obj.getClass() != this.getClass()) return false;
            FeedForwardGains that = (FeedForwardGains) obj;
            return Double.doubleToLongBits(this.kS) == Double.doubleToLongBits(that.kS) &&
                    Double.doubleToLongBits(this.kV) == Double.doubleToLongBits(that.kV) &&
                    Double.doubleToLongBits(this.kA) == Double.doubleToLongBits(that.kA);
        }

    }

    public static final class GravityFeedForwardGains implements Gains{
        private final double kS;
        private final double kV;
        private final double kA;
        private final double kG;

        public GravityFeedForwardGains(double kS, double kV, double kG, double kA) {
            this.kS = kS;
            this.kV = kV;
            this.kA = kA;
            this.kG = kG;
        }

        public double kS() {
            return kS;
        }

        public double kV() {
            return kV;
        }

        public double kA() {
            return kA;
        }

        public double kG() {
            return kG;
        }

        @Override
        public double[] toArray(){
            return new double[]{kS,kV,kG, kA};
        }

        @Override
        public Gains fromArray(double[] array){
            return new GravityFeedForwardGains(array[0], array[1], array[2], array[3]);
        }

        @Override
        public boolean equals(Object obj) {
            if (obj == this) return true;
            if (obj == null || obj.getClass() != this.getClass()) return false;
            GravityFeedForwardGains that = (GravityFeedForwardGains) obj;
            return Double.doubleToLongBits(this.kS) == Double.doubleToLongBits(that.kS) &&
                    Double.doubleToLongBits(this.kV) == Double.doubleToLongBits(that.kV) &&
                    Double.doubleToLongBits(this.kA) == Double.doubleToLongBits(that.kA) &&
                    Double.doubleToLongBits(this.kG) == Double.doubleToLongBits(that.kG);
        }
    }

    public static final class TrapezoidalGains implements Gains{
        private final double kP;
        private final double kI;
        private final double kD;

        private final double maxVelocity;

        private final double maxAcceleration;

        public TrapezoidalGains(
                double kP, double kI, double kD,
                double maxVelocity, double maxAcceleration) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            this.maxVelocity = maxVelocity;
            this.maxAcceleration = maxAcceleration;
        }

        public double kP() {
            return kP;
        }

        public double kI() {
            return kI;
        }

        public double kD() {
            return kD;
        }

        public double maxAcceleration() {
            return maxAcceleration;
        }

        public double maxVelocity() {
            return maxVelocity;
        }

        @Override
        public double[] toArray(){
            return new double[]{kP, kI,kD,maxVelocity, maxAcceleration};
        }

        @Override
        public TrapezoidalGains fromArray(double[] array){
            return new TrapezoidalGains(
                    array[0], array[1], array[2], array[3], array[4]
            );
        }

        @Override
        public boolean equals(Object obj) {
            if (obj == this) return true;
            if (obj == null || obj.getClass() != this.getClass()) return false;
            TrapezoidalGains that = (TrapezoidalGains) obj;
            return Double.doubleToLongBits(this.kP) == Double.doubleToLongBits(that.kP) &&
                    Double.doubleToLongBits(this.kI) == Double.doubleToLongBits(that.kI) &&
                    Double.doubleToLongBits(this.kD) == Double.doubleToLongBits(that.kD) &&
                    Double.doubleToLongBits(this.maxAcceleration) == Double.doubleToLongBits(that.maxAcceleration) &&
                    Double.doubleToLongBits(this.maxVelocity) == Double.doubleToLongBits(that.maxVelocity);
        }

    }
}