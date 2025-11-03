package org.firstinspires.ftc.teamcode.chuckFull.opmodes;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.seattlesolvers.solverslib.command.CommandBase;

import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import org.firstinspires.ftc.teamcode.chuckFull.RobotContainer;


@TeleOp(name = "TeleOp - Modular Commands", group = "Main")
public class TeleOpMain extends LinearOpMode {
    private RobotContainer robot;
    private GamepadEx driver;


    @Override
    public void runOpMode() {
        driver = new GamepadEx(gamepad1);
        robot = new RobotContainer(hardwareMap, driver);


        waitForStart();


        while (opModeIsActive()) {
            driver.readButtons();
            robot.run();
            telemetry.update();
        }
    }
}