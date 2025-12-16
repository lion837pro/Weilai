package org.firstinspires.ftc.teamcode.Robot.Hardware;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import dev.nextftc.ftc.ActiveOpMode;

public class REV312010 {

    private final DigitalChannel[] redLEDs;
    private final DigitalChannel[] greenLEDs;
    private boolean initialized = false;
    private int redCount = 0;
    private int greenCount = 0;

    public enum LEDState{
        kGREEN, kRED, kAMBER, kOFF
    }

    private LEDState currentState;

    public REV312010(){
        HardwareMap map = ActiveOpMode.hardwareMap();

        // Support up to 4 LED channels (will gracefully handle if fewer are configured)
        redLEDs = new DigitalChannel[4];
        greenLEDs = new DigitalChannel[4];

        // Try to initialize 4 red and 4 green LED channels
        for (int i = 0; i < 4; i++) {
            try {
                String redName = (i == 0) ? "red" : "red" + (i + 1);
                redLEDs[i] = map.get(DigitalChannel.class, redName);
                redLEDs[i].setMode(DigitalChannel.Mode.OUTPUT);
                redCount++;
            } catch (Exception e) {
                redLEDs[i] = null; // LED not configured
            }

            try {
                String greenName = (i == 0) ? "green" : "green" + (i + 1);
                greenLEDs[i] = map.get(DigitalChannel.class, greenName);
                greenLEDs[i].setMode(DigitalChannel.Mode.OUTPUT);
                greenCount++;
            } catch (Exception e) {
                greenLEDs[i] = null; // LED not configured
            }
        }

        initialized = (redCount > 0 || greenCount > 0);
        this.currentState = LEDState.kOFF;

        // Log initialization status
        ActiveOpMode.telemetry().addData("LED Init", "Red: %d, Green: %d", redCount, greenCount);
    }

    /**
     * Check if any LEDs were successfully initialized
     */
    public boolean isInitialized() {
        return initialized;
    }

    /**
     * Get count of initialized red LEDs
     */
    public int getRedCount() {
        return redCount;
    }

    /**
     * Get count of initialized green LEDs
     */
    public int getGreenCount() {
        return greenCount;
    }

    public void set(LEDState state){
        switch(state){
            case kGREEN:
                this.currentState = LEDState.kGREEN;
                setState(false,true); // Green LOW (ON), Red HIGH (OFF)
                break;
            case kRED:
                this.currentState = LEDState.kRED;
                setState(true,false); // Green HIGH (OFF), Red LOW (ON)
                break;
            case kAMBER:
                this.currentState = LEDState.kAMBER;
                setState(false,false); // Both LOW (ON)
                break;
            default:
                this.currentState = LEDState.kOFF;
                setState(true,true); // Both HIGH (OFF)
                break;
        }
    }

    public LEDState get(){
        return this.currentState;
    }

    private void setState(boolean green, boolean red){
        // Set all 4 LEDs to the same state
        for (int i = 0; i < 4; i++) {
            if (greenLEDs[i] != null) {
                greenLEDs[i].setState(green);
            }
            if (redLEDs[i] != null) {
                redLEDs[i].setState(red);
            }
        }
    }

}