package org.firstinspires.ftc.teamcode.testing;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "sensor")
public class sensorTest extends OpMode {
    private NormalizedColorSensor colour;
    private DistanceSensor distance;
    private final float[] hsv = new float[3];
    @Override
    public void init() {



        colour = hardwareMap.get(NormalizedColorSensor.class,"colour1");
        distance = hardwareMap.get(DistanceSensor.class, "colour1");
        colour.setGain(2.0f);
    }

    @Override
    public void loop(){

        NormalizedRGBA rgba = colour.getNormalizedColors();
        Color.colorToHSV(rgba.toColor(), hsv);

        telemetry.addLine("RAW (normalized)")
                .addData("R", "%.3f", rgba.red)
                .addData("G", "%.3f", rgba.green)
                .addData("B", "%.3f", rgba.blue)
                .addData("A", "%.3f", rgba.alpha);
        telemetry.addLine("HSV")
                .addData("H (deg)", "%.1f", hsv[0])
                .addData("S", "%.3f", hsv[1])
                .addData("V", "%.3f", hsv[2]);
        telemetry.addData("Distance (cm)", "%.2f", distance.getDistance(DistanceUnit.CM));
        telemetry.addLine(" ");
        if (distance.getDistance(DistanceUnit.CM) <=2.5) {
            if ((150 <= hsv[0] && hsv[0] <= 180) && (0.75 <= hsv[1] && hsv[1] <= 1.0) && (0 < hsv[2] && hsv[2] < 0.16)) {
                telemetry.addLine("its green");
            } else if ((220 <= hsv[0] && hsv[0] <= 250) && (0.4 <= hsv[1] && hsv[1] <= 0.6) && (0 < hsv[2] && hsv[2] < 0.16)) {
                telemetry.addLine("purpel");
            }
        }
        telemetry.update();
    }
}