package org.firstinspires.ftc.teamcode.Turret;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.seattlesolvers.solverslib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.var;

@TeleOp
public class WheelPID extends OpMode {
    private double RPM;
    private double lastTime;
    private int lastPosition;
    Motor launcher;
    FtcDashboard dashboard = FtcDashboard.getInstance();


    @Override
    public void init() {
        Motor launcher = new Motor(hardwareMap, "launcher");
        launcher.setRunMode(Motor.RunMode.RawPower);

        lastTime = getRuntime();
        lastPosition = launcher.getCurrentPosition();
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

    public void runFeedforward () {
            SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(var.fwKs, var.fwKv, var.fwKa);

        double targetRpm = 5000.0;
        double feedforwardPower = feedforward.calculate(targetRpm, 0.0); // accel = 0 at steady-state

        launcher.set(feedforwardPower);
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
    }

}
