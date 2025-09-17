package org.firstinspires.ftc.teamcode;



import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

@TeleOp
public class turrettest extends OpMode {
    private DcMotor motor;
    private double RPM;
    private double lastTime;
    private int lastPosition;

    FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotor.class, "launcher");
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lastTime = getRuntime();
        lastPosition = motor.getCurrentPosition();
    }

    @Override
    public void loop() {
        // Calculate RPM
        double currentTime = getRuntime();
        int currentPosition = motor.getCurrentPosition();

        double deltaTime = currentTime - lastTime;
        int deltaTicks = currentPosition - lastPosition;

        if (deltaTime > 0.1) {
            double revs = (double) deltaTicks / 28; // GoBILDA CPR
            RPM = (revs / deltaTime) * 60.0;

            lastTime = currentTime;
            lastPosition = currentPosition;
        }
        motor.setPower(1);

        // Normal telemetry
        telemetry.addData("RPM", RPM);
        telemetry.update();

        // Dashboard graph
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("RPM", RPM);
        dashboard.sendTelemetryPacket(packet);
    }
}
