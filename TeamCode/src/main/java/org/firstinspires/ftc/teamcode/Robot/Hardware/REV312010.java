package org.firstinspires.ftc.teamcode.Robot.Hardware;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import dev.nextftc.ftc.ActiveOpMode;

public class REV312010 {

    private final DigitalChannel[] redLEDs;
    private final DigitalChannel[] greenLEDs;

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
            } catch (Exception e) {
                redLEDs[i] = null; // LED not configured
            }

            try {
                String greenName = (i == 0) ? "green" : "green" + (i + 1);
                greenLEDs[i] = map.get(DigitalChannel.class, greenName);
                greenLEDs[i].setMode(DigitalChannel.Mode.OUTPUT);
            } catch (Exception e) {
                greenLEDs[i] = null; // LED not configured
            }
        }

        this.currentState = LEDState.kOFF;
    }

    public void set(LEDState state){
        switch(state){
            case kGREEN:
                this.currentState = LEDState.kGREEN;
                setState(true,false);
                break;
            case kRED:
                this.currentState = LEDState.kRED;
                setState(false,true);
                break;
            case kAMBER:
                this.currentState = LEDState.kAMBER;
                setState(true,true);
                break;
            default:
                this.currentState = LEDState.kOFF;
                setState(false,false);
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