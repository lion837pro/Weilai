package org.firstinspires.ftc.teamcode.Lib;
import com.qualcomm.robotcore.util.ElapsedTime;
/**
 * A slew rate limiter limits the rate of change of a value over time.
 * This prevents sudden jumps in motor power that can cause belt slipping.
 */

    public class SlewRateLimiter {
        private final double maxRateOfChange; // Maximum change per second
        private double previousValue;
        private final ElapsedTime timer;
        private double lastTime;

        /**
         * Creates a new SlewRateLimiter
         *
         * @param maxRateOfChange Maximum rate of change per second
         *                        Example: 2.0 means it takes 0.5 seconds to go from 0 to 1.0 power
         */
        public SlewRateLimiter(double maxRateOfChange) {
            this.maxRateOfChange = maxRateOfChange;
            this.previousValue = 0.0;
            this.timer = new ElapsedTime();
            this.lastTime = 0.0;
        }

        /**
         * Applies the slew rate limit to the input value
         *
         * @param input The desired value
         * @return The rate-limited value
         */
        public double calculate(double input) {
            // Calculate time delta
            double currentTime = timer.seconds();
            double dt = currentTime - lastTime;
            lastTime = currentTime;

            // Prevent division by zero or negative dt
            if (dt <= 0) {
                return previousValue;
            }

            // Calculate maximum allowed change
            double maxChange = maxRateOfChange * dt;

            // Calculate actual change needed
            double change = input - previousValue;

            // Limit the change
            if (Math.abs(change) > maxChange) {
                change = Math.copySign(maxChange, change);
            }

            // Apply the limited change
            previousValue = previousValue + change;

            return previousValue;
        }

        /**
         * Resets the limiter to a specific value
         * Useful when you want to start from a known state
         */
        public void reset(double value) {
            this.previousValue = value;
            this.timer.reset();
            this.lastTime = 0.0;
        }

        /**
         * Gets the current output value
         */
        public double getLastValue() {
            return previousValue;
        }
    }

