//package org.firstinspires.ftc.teamcode.chuckFull;
//
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.seattlesolvers.solverslib.command.CommandOpMode;
//import com.seattlesolvers.solverslib.gamepad.GamepadEx;
//
//@TeleOp(name = "Main TeleOp", group = "ChuckFull")
//public class MainTeleOp extends CommandOpMode {
//    private RobotContainer robot;
//    private GamepadEx driver1;
//    private GamepadEx driver2;
//
//    @Override
//    public void initialize() {
//        // Initialize robot subsystems
//        robot = new RobotContainer(hardwareMap);
//
//        // Wrap the built-in FTC gamepads
//        driver1 = new GamepadEx(gamepad1);
//        driver2 = new GamepadEx(gamepad2);
//
//        // Schedule your driver-control commands
//        schedule(robot.getDriveCommand(driver1));
//        schedule(robot.getRevolverCommand(driver2));
//
//        telemetry.addLine("Main TeleOp initialized");
//        telemetry.update();
//    }
//
//    @Override
//    public void run() {
//        // Let SolversLib handle the command scheduler
//        super.run();
//
//    }
//}
