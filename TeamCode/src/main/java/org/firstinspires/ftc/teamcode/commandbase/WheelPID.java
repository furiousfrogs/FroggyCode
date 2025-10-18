package org.firstinspires.ftc.teamcode.commandbase;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.hardware.SimpleServo;
import com.seattlesolvers.solverslib.hardware.motors.CRServo;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.hardware.Globals;

@TeleOp
public class WheelPID extends OpMode {
    private double RPM;
    private double lastTime;
    private int lastPosition;





    Motor launcher1, launcher2;
    SimpleServo set;
    CRServo rotate;
    FtcDashboard dashboard = FtcDashboard.getInstance();

    GamepadEx gamepadEx;


    @Override
    public void init() {
        launcher1 = new Motor(hardwareMap, "fl", 28, 6000);
        launcher1.setRunMode(Motor.RunMode.RawPower);

        launcher2 = new Motor(hardwareMap, "fr", 28, 6000);
        launcher2.setRunMode(Motor.RunMode.RawPower);

        set = new SimpleServo(
                hardwareMap, "set", 0, 180,
                AngleUnit.DEGREES
        );

        

        lastTime = getRuntime();
        lastPosition = launcher1.getCurrentPosition();

        gamepadEx = new GamepadEx(gamepad1);


    }

    public void calculateRPM() {
        double currentTime = getRuntime();
        int currentPosition = launcher1.getCurrentPosition();

        double deltaTime = currentTime - lastTime;
        int deltaTicks = currentPosition - lastPosition;

        if (deltaTime > 0.1) {
            double revs = (double) deltaTicks / 28; // GoBILDA CPR
            RPM = (revs / deltaTime) * 60.0;

            lastTime = currentTime;
            lastPosition = currentPosition;
        }
    }

    public void runFeedforward() {
        SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Globals.fwKs, Globals.fwKv, Globals.fwKa);


        double feedforwardPower = feedforward.calculate(Globals.targetrpm, 0.0); // accel = 0 at steady-state


        if (gamepadEx.getButton(GamepadKeys.Button.CROSS)) {
            launcher1.set(feedforwardPower);
            launcher2.set(feedforwardPower);
            if (Globals.targetrpm > 1000 && Math.abs(Globals.targetrpm - RPM) < Globals.launcherTol) { // TODO replace targetrpm with power
                set.turnToAngle(Globals.upset);
            }
        } else if (gamepadEx.getButton(GamepadKeys.Button.TRIANGLE)) {
            launcher1.set(0.9);
            launcher2.set(0.9);
            set.turnToAngle(Globals.downset);
        } else if (!gamepadEx.getButton(GamepadKeys.Button.TRIANGLE) || !gamepadEx.getButton(GamepadKeys.Button.CROSS)){
            launcher1.set(0);
            launcher2.set(0);
            set.turnToAngle(Globals.downset);
        }

    }



    public void telemetry() {
        // Normal telemetry
        telemetry.addData("RPM", RPM);
        telemetry.update();

        // Dashboard graph
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("RPM", RPM);
        dashboard.sendTelemetryPacket(packet);
    }

    @Override
    public void loop() {
        calculateRPM();
        runFeedforward();
        telemetry();


    }
}
