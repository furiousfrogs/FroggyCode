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
import com.seattlesolvers.solverslib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.hardware.Globals;

@TeleOp
public class WheelPID extends OpMode {
    private double RPM;
    private double lastTime;
    private int lastPosition;





    Motor launcher;
    SimpleServo set;
    FtcDashboard dashboard = FtcDashboard.getInstance();

    GamepadEx gamepadEx;


    @Override
    public void init() {
        launcher = new Motor(hardwareMap, "fl", 28, 6000);
        launcher.setRunMode(Motor.RunMode.RawPower);

        set = new SimpleServo(
                hardwareMap, "set", 0, 180,
                AngleUnit.DEGREES
        );

        lastTime = getRuntime();
        lastPosition = launcher.getCurrentPosition();

        gamepadEx = new GamepadEx(gamepad1);
        GamepadButton triangle = new GamepadButton(
                gamepadEx, GamepadKeys.Button.TRIANGLE
        );
        GamepadButton circle = new GamepadButton(
                gamepadEx, GamepadKeys.Button.CIRCLE
        );

    }

    public void calculateRPM() {
        double currentTime = getRuntime();
        int currentPosition = launcher.getCurrentPosition();

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

        launcher.set(feedforwardPower);
    }

    public void adjusttarget() {
        if(gamepadEx.getButton(GamepadKeys.Button.CIRCLE)){
            set.turnToAngle(Globals.downset);
        } else if (gamepadEx.getButton(GamepadKeys.Button.TRIANGLE)) {
            set.turnToAngle(Globals.upset);
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
        adjusttarget();
    }
}
