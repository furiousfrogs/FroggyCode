package org.firstinspires.ftc.teamcode.commandbase;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.seattlesolvers.solverslib.controller.PDController; // SolversLib PD

import org.firstinspires.ftc.teamcode.hardware.Globals;

@TeleOp(name = "xx", group = "Frog")
public class xx extends LinearOpMode {

    // --- Minimal constants you MUST set correctly ---
    private static final double TICKS_PER_REV = 537.7;   // TODO: your motor encoder CPR
    private static final double OUTPUT_GEAR_RATIO = 1.0; // outputRotations / motorRotations

    // PD gains (start here, then tune)
    private static final double kP = Globals.revolverKP;   // TODO tune on-robot
    private static final double kD = Globals.revolverKD;  // TODO tune on-robot

    // Basic limits/finish
    private static final double MAX_POWER = 0.6;
    private static final int    TOL_TICKS = Globals.revolverTol;

    private DcMotorEx revolver;
    private PDController pd;
    private double targetTicks;
    private int zeroTicks;

    @Override
    public void runOpMode() {
        revolver = hardwareMap.get(DcMotorEx.class, "revolver");
        revolver.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        revolver.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        revolver.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pd = new PDController(kP, kD); // SolversLib PD
        zeroTicks = revolver.getCurrentPosition();
        targetTicks = zeroTicks;

        telemetry.addLine("A:+360  B:-360  X:zero");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            pd.setP(Globals.revolverKP);
            pd.setD(Globals.revolverKD);
            pd.setTolerance(Globals.revolverTol);
            // --- Adjust target ---
            if (gamepad1.a) { targetTicks += ticksForDegrees(360); sleep(150); }
            if (gamepad1.b) { targetTicks -= ticksForDegrees(360); sleep(150); }
            if (gamepad1.x) { targetTicks  = zeroTicks;            sleep(150); }

            // --- PD control with SolversLib ---
            int current = revolver.getCurrentPosition();
            pd.setSetPoint(targetTicks); // set desired encoder ticks
            double output = pd.calculate(current); // PD output (power-ish)

            // Clamp, apply, and stop near target
            double error = targetTicks - current;
            double power = clamp(output, -MAX_POWER, MAX_POWER);
            if (Math.abs(error) <= TOL_TICKS) power = 0.0;

            revolver.setPower(power);

            // Minimal telemetry for tuning
            telemetry.addData("Target", (int)Math.round(targetTicks));
            telemetry.addData("Current", current);
            telemetry.addData("Error", (int)Math.round(error));
            telemetry.addData("Power", "%.3f", power);
            telemetry.update();

            sleep(15);
        }
        revolver.setPower(0);
    }

    private static double ticksForDegrees(double deg) {
        double ticksPerOutputRev = TICKS_PER_REV / OUTPUT_GEAR_RATIO;
        return ticksPerOutputRev * (deg / 360.0);
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
;