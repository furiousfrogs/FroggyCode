package org.firstinspires.ftc.teamcode.testing;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.seattlesolvers.solverslib.hardware.SimpleServo;
import com.seattlesolvers.solverslib.hardware.SimpleServoExtKt;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.Globals;

@TeleOp(name = "sensor")
public class sensorTest extends OpMode {
    private DistanceSensor distanceSensor;
    private AnalogInput ejectAnalog;
    private SimpleServo eject;
    @Override
    public void init() {
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        ejectAnalog = hardwareMap.get(AnalogInput.class, "ejectAnalog");
        eject = new SimpleServo(hardwareMap, "eject", 0, 70);
        eject.setInverted(true);
        eject.turnToAngle(Globals.pushServo.defualt);
    }

    @Override
    public void loop(){
     telemetry.addData("distance, ", distanceSensor.getDistance(DistanceUnit.CM));
        telemetry.addData("voltage, ", (ejectAnalog.getVoltage()/3.3)*360);
        if (gamepad2.right_stick_x > 0.5) {
            eject.turnToAngle(Globals.pushServo.eject);
        } else if (gamepad2.right_stick_x < -0.5) {
            eject.turnToAngle(Globals.pushServo.push);
        } else {
            eject.turnToAngle(Globals.pushServo.defualt);

        }// eject is 30, default is 44, push is 51

        telemetry.update();
    }
}