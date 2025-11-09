package org.firstinspires.ftc.teamcode.hardware;


import com.acmerobotics.dashboard.config.Config;

@Config
public class Globals {

    @Config
    public static class turret {
        public static float turretTol = 4F;
        public static float nudge = 5F;
        public static double turretKP = 0.17F;
        public static double turretKI = 0.000F;
        public static double turretKD = 0.01F;
        public static double turretKF = 0.000F;
        public static float turretLocationError = -1F;

        public static float targetRPM = 3000F;
    }

    @Config
    public static class revolver {
        public static int oneRotation = 128; //|| Minus is counter-clockwise rotation.
        public static float revolverKP = 0.007F;
        public static float revolverKD= 0.0003F;
        public static float revolverKI = 0F;
        public static float revolverKF = 0;
        public static float revolverNudge = 2F;
    }

    @Config
    public static class launcher {
        public static float RPMDipThreshold = 200;
        public static float launcherTransformation = 1.06F;
        public static float launcherTol = 100F;
        public static double flykP =0.00016811;
        public static double flykI =0.0F;
        public static double flykD = 0.0F ;
        public static double flykF = 0.000020;

        public static double downset=0.0F;
        public static double upset=300;

    }
        public static float intakePower = 0.7F;

    @Config
    public static class timers {
        public static float servoPushTime = 1F;
        public static float oneRotationTime = 1F;
        public static float pushUpTime = 1F;
    }

    @Config // eject is 30, default is 44, push is 51
    public static class pushServo {
        public static float push = 30F;
        public static float defualt = 17F;
        public static float eject = 0F;
    }

    public static float slowdown = 0.6F;
    public static float revolverPower = 0;


}