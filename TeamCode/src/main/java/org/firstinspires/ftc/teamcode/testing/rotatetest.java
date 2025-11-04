package org.firstinspires.ftc.teamcode.testing;

import com.pedropathing.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.hardware.SimpleServo;
import com.seattlesolvers.solverslib.hardware.SimpleServoExtKt;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.hardware.Globals;
@Disabled
@TeleOp(name = "rotate test")
public class rotatetest extends OpMode {
    private Motor revolver, intake;
    private GamepadEx gamepadEx;

    private boolean indexInProgress = false;
    private int revolverTarget = 0;

    private PIDController revolverPID;

    private double revolverPower;

    private SimpleServo eject;
    // tuning
    private static final double DRIVE_POWER = 0.1; // while PositionControl does its job

    @Override
    public void init() {
        revolver = new Motor(hardwareMap, "revolver", 384.5, 435);
        revolver.setRunMode(Motor.RunMode.RawPower);

        intake = new Motor(hardwareMap, "intake");
        intake.setRunMode(Motor.RunMode.RawPower);
        intake.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        revolver.resetEncoder();                     // zero now (or after homing)


        gamepadEx = new GamepadEx(gamepad1);

        revolverPID = new PIDController(Globals.revolver.revolverKP, Globals.revolver.revolverKI, Globals.revolver.revolverKD);
        eject = new SimpleServo(hardwareMap, "eject", 0, 70);
        eject.setInverted(true);


    }

    @Override
    public void loop() {
        //rotate();
        feedfoward();

        // eject is 30, default is 44, push is 55
        //rotate();
        //intake();
        // (optional) telemetry
        telemetry.addData("currently: ", revolver.getCurrentPosition());
        telemetry.addData("target: ", revolverTarget);
        telemetry.update();
    }
    public void feedfoward() {
        if (gamepadEx.getButton(GamepadKeys.Button.SQUARE)) {
            revolver.set(Globals.revolverPower);
        } else { revolver.set(0); }
    }

    public void rotate() {
        revolverPID.setTolerance(0);
        gamepadEx.readButtons();
        revolverPID.setPID(Globals.revolver.revolverKP, Globals.revolver.revolverKI, Globals.revolver.revolverKD);

        if (gamepadEx.wasJustPressed(GamepadKeys.Button.SQUARE)) {
            revolverTarget += Globals.revolver.oneRotation;  // CW
        } else if (gamepadEx.wasJustPressed(GamepadKeys.Button.CROSS)) {
            revolverTarget -= Globals.revolver.oneRotation;
        }

        revolverPower = revolverPID.calculate(revolver.getCurrentPosition(), revolverTarget);
        revolver.set(revolverPower);

    }
    public void intake() {

        if (gamepadEx.getButton(GamepadKeys.Button.TRIANGLE)) {
            intake.set(Globals.intakePower);
        } else if (gamepadEx.getButton(GamepadKeys.Button.SQUARE)) {
            intake.set(-Globals.intakePower);
        } else { intake.set(0); }
    }
}
