package org.firstinspires.ftc.teamcode.chuckFull;


import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.chuckFull.subsystems.*;
import org.firstinspires.ftc.teamcode.chuckFull.commands.*;


public class RobotContainer {
    public final DrivebaseSubsystem drivebase;
    public final LauncherSubsystem launcher;
    public final RevolverSubsystem revolver;
    public final TurretSubsystem turret;
    public final IntakeSubsystem intake;
    public final PatternSubsystem pattern;
    public final GamepadEx driver;


    public RobotContainer(HardwareMap hw, GamepadEx driver) {
        this.driver = driver;


// instantiate all subsystems
        drivebase = new DrivebaseSubsystem(hw);
        launcher = new LauncherSubsystem(hw);
        revolver = new RevolverSubsystem(hw);
        turret = new TurretSubsystem(hw);
        intake = new IntakeSubsystem(hw);
        pattern = new PatternSubsystem(hw);


// set default commands
        CommandScheduler.getInstance().setDefaultCommand(drivebase, new DriveCommand(drivebase,driver));
        CommandScheduler.getInstance().setDefaultCommand(launcher, new LauncherCommand(launcher, turret, pattern, driver));
        CommandScheduler.getInstance().setDefaultCommand(revolver, new RevolverCommand(revolver, driver));
        CommandScheduler.getInstance().setDefaultCommand(turret, new TurretAutoAimCommand(turret, driver));
        CommandScheduler.getInstance().setDefaultCommand(intake, new IntakeCommand(intake, driver));
        CommandScheduler.getInstance().setDefaultCommand(pattern, new PatternDetectCommand(pattern, driver));
    }


    public void run() {
        CommandScheduler.getInstance().run();
    }
}