package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;


import java.util.List;
@TeleOp(name = "LLLLLL")
public class limelighttest extends OpMode {
    Limelight3A limelight;
    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(0);
    }

    @Override
    public void loop() {
//        if (gamepadEx2.getButton(GamepadKeys.Button.OPTIONS)) {
//            limelight.start();
//            LLResult result = limelight.getLatestResult();
//            if (result != null && result.isValid()) {
//
//                List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
//                if (!tags.isEmpty()) {
//                    for (LLResultTypes.FiducialResult tag : tags) {
//                        int id = tag.getFiducialId();
//                        if (id == 21) {
//                            currentPattern = teleManual.pattern.GPP;
//                            patternDetected = true;
//                            limelight.stop();
//                        } else if (id == 22) {
//                            currentPattern = teleManual.pattern.PGP;
//                            patternDetected = true;
//                            limelight.stop();
//                        } else if (id == 23) {
//                            currentPattern = teleManual.pattern.PPG;
//                            patternDetected = true;
//                            limelight.stop();
//                        } else {
//                            patternDetected = false;
//                            telemetry.addLine("NO PATTERN FOUND");
//                        }
//
//
//                    }
//                }
//            }
//        }
    }
}
