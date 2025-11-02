package org.firstinspires.ftc.teamcode.chuckFull.subsystems;


import android.graphics.Color;

import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Objects;




public class RevolverSubsystem extends SubsystemBase {
    private final Motor revolver;
    private final NormalizedColorSensor colorSensor;
    private final DistanceSensor distanceSensor;
    private final PIDController pid;


    public List<String> revolverState = new ArrayList<>(Arrays.asList("EMPTY", "EMPTY", "EMPTY"));
    public boolean revolverReady = true;
    private double previousRevolverPosition = 0;
    private int revolverTarget = 0;
    private NormalizedColorSensor colourSensor;


    private final float[] hsv = new float[3];



    public RevolverSubsystem(HardwareMap hw) {
        revolver = new Motor(hw, "revolver", 28, 1150);
        revolver.setRunMode(Motor.RunMode.RawPower);
        revolver.resetEncoder();
        pid = new PIDController(Globals.revolver.revolverKP, Globals.revolver.revolverKI, Globals.revolver.revolverKD);


        colorSensor = hw.get(NormalizedColorSensor.class, "colour1");
        distanceSensor = hw.get(DistanceSensor.class, "colour1");
        colorSensor.setGain(2.0f);
    }


    public void periodic(GamepadEx g) {
// simple intake control
        int filled = revolverState.size() - Collections.frequency(revolverState, "EMPTY");
        String color = senseColour();


        if (g.getButton(com.seattlesolvers.solverslib.gamepad.GamepadKeys.Button.SQUARE) && revolverReady && !"EMPTY".equals(color)) {
            revolverReady = false;
// basic insert logic: put sensed color into rightmost chamber and rotate depending on state
            revolverState.set(2, color);
        }
    }
    private String senseColour() {
        if (distanceSensor.getDistance(DistanceUnit.CM) > 3) return "EMPTY";

        NormalizedRGBA rgba = colourSensor.getNormalizedColors();
        Color.colorToHSV(rgba.toColor(), hsv); // hsv[0]=H, hsv[1]=S, hsv[2]=V

        if (hsv[0] >= 150 && hsv[0] <= 180 &&
                hsv[1] >= 0.75 && hsv[1] <= 1.00 &&
                hsv[2] > 0.00 && hsv[2] < 0.3) {
            return "G";
        }

        if (hsv[0] >= 220 && hsv[0] <= 250 &&
                hsv[1] >= 0.40 && hsv[1] <= 0.60 &&
                hsv[2] > 0.00 && hsv[2] < 0.3) {
            return "P";
        }
        return "EMPTY";
    }

}