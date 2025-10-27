package org.firstinspires.ftc.teamcode.deprsTest;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

@TeleOp(name = "Sample")
public class CommandTest extends CommandOpMode {
    @Override
    public void initialize() {

    }

    public class ejectSubsystem extends SubsystemBase {

        private ElapsedTime globalTimer = new ElapsedTime();
        private double servoTimer;
        private boolean servoAction;
        private ServoEx eject;
        public ejectSubsystem() {
            eject = new ServoEx(hardwareMap, "eject", 0, 70);
            eject.set(44);
        }

        public void pushAction() {
            eject.set(51);
        }
        public void ejectAction() {
            eject.set(28);
        }

        public void defaultAction() {
            eject.set(44);
        }// eject is 30, default is 44, push is 51
    }

    public class ejectServo extends CommandBase {
        private ejectSubsystem ejectServo;

        private GamepadEx driver1, driver2;

        @Override
        public void initialize() {
            ejectServo = new ejectSubsystem();


        }
    }
}