//package org.firstinspires.ftc.teamcode.chuckFull;
//
//import org.firstinspires.ftc.teamcode.chuckFull.subsystems.*;
//import org.firstinspires.ftc.teamcode.chuckFull.commands.*;
//
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.seattlesolvers.solverslib.gamepad.GamepadEx;
//
//
//@
//public class RobotContainer {
//    public final Drivebase drivebase;
//    public final RevolverSystem revolverSystem;
//
//    public RobotContainer(HardwareMap hwMap) {
//        drivebase = new Drivebase(hwMap);
//        revolverSystem = new RevolverSystem(hwMap);
//    }
//
//    // Just return the commands; don't try to register them here
//    public DriveCommand getDriveCommand(GamepadEx gamepad1) {
//        return new DriveCommand(drivebase, gamepad1);
//    }
//
//    public RevolverCommand getRevolverCommand(GamepadEx gamepad2) {
//        return new RevolverCommand(revolverSystem, gamepad2);
//    }
//}
