package org.firstinspires.ftc.teamcode.chuckFull.subsystems;


import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


import java.util.Comparator;
import java.util.List;


public class PatternSubsystem extends SubsystemBase {
    public enum Pattern { PPG, PGP, GPP }
    public boolean patternDetected = false;
    public Pattern currentPattern = Pattern.PPG;


    private final AprilTagProcessor tagProcessor;


    public PatternSubsystem(com.qualcomm.robotcore.hardware.HardwareMap hw) {
        tagProcessor = new AprilTagProcessor.Builder().build();
    }


    public void periodic(GamepadEx g) {
        if (g.getButton(com.seattlesolvers.solverslib.gamepad.GamepadKeys.Button.OPTIONS)) {
            List<AprilTagDetection> code = tagProcessor.getDetections();
            if (code != null && !code.isEmpty()) {
                code.sort(Comparator.comparingDouble((AprilTagDetection d) -> d.decisionMargin).reversed());
                AprilTagDetection best = code.get(0);
                if (best.id == 21) { currentPattern = Pattern.GPP; patternDetected = true; }
                else if (best.id == 22) { currentPattern = Pattern.PGP; patternDetected = true; }
                else if (best.id == 23) { currentPattern = Pattern.PPG; patternDetected = true; }
                else { patternDetected = false; }
            }
        }
    }
}