package org.firstinspires.ftc.teamcode.Robot.Hardware;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import dev.nextftc.ftc.ActiveOpMode;

public class REV312010 {

    private final DigitalChannel redLED;
    private final DigitalChannel greenLED;

    public enum LEDState{
        kGREEN, kRED, kAMBER, kOFF
    }

    private LEDState currentState;

    public REV312010(){
        HardwareMap map = ActiveOpMode.hardwareMap();
        redLED = map.get(DigitalChannel.class, "red");
        greenLED = map.get(DigitalChannel.class, "green");

        redLED.setMode(DigitalChannel.Mode.OUTPUT);
        greenLED.setMode(DigitalChannel.Mode.OUTPUT);

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
        greenLED.setState(green);
        redLED.setState(red);
    }

}