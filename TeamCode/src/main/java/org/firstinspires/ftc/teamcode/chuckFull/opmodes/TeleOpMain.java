package org.firstinspires.ftc.teamcode.chuckFull.opmodes;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import org.firstinspires.ftc.teamcode.chuckFull.RobotContainer;


@TeleOp(name = "Chuck Tele", group = "Main")
public class TeleOpMain extends LinearOpMode {



    @Override
    public void runOpMode() {
        RobotContainer robot;
        GamepadEx driver;

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