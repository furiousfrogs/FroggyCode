package org.firstinspires.ftc.teamcode.deprsTest;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
@TeleOp(name = "niggerrun")
public class bareMotorTest extends OpMode {
    private double lastTime, previousRPM, RPM;
    private int lastPosition;
    private Motor nigger;
    @Override
    public void init() {
        nigger = new Motor(hardwareMap, "motor");
    }

    @Override
    public void loop() {

        double currentTime = getRuntime();
        int currentPosition = nigger.getCurrentPosition();

        double deltaTime = currentTime - lastTime;
        double deltaTicks = currentPosition - lastPosition;

        if (deltaTime > 0.02) {
            previousRPM = RPM;
            double revs = (double) deltaTicks / 28.0; // GoBILDA CPR
            RPM = (revs / deltaTime) * 60.0;

            lastTime = currentTime;
            lastPosition = currentPosition;
        }
        telemetry.addData("RPM", RPM);
        nigger.set(1);
    }
}
