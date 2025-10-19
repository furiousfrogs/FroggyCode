package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.drivebase.MecanumDrive;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

@TeleOp
public class solversdrive extends OpMode {

    private Motor fl, fr, bl, br;
    private MecanumDrive drive;
    private GamepadEx driverOp;

    @Override
    public void init() {
        /* instantiate motors */

        drive = new MecanumDrive(fl, fr, bl, br);
        driverOp = new GamepadEx(gamepad1);
    }

    @Override
    public void loop() {
        drive.driveRobotCentric(
                driverOp.getLeftX(),
                driverOp.getLeftY(),
                driverOp.getRightY()
        );
    }

}