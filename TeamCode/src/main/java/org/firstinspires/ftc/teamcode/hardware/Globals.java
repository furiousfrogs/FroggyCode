package org.firstinspires.ftc.teamcode.hardware;


import com.acmerobotics.dashboard.config.Config;

@Config
public class Globals {

    @Config
    public static class turret {
        public static float turretTol = 2F;
        public static float nudge = 1F;
        public static double turretKP = 0.018F;
        public static double turretKI = 0.000F;
        public static double turretKD = 0.013F;
        public static double turretKF = 0.000F;
        public static float turretLocationError = -1F;

        public static float targetRPM = 3000F;
    }

    @Config
    public static class revolver {
        public static int oneRotation = 128; // actually 2 rotations lol  || Minus is counter-clockwise rotation.
        public static float revolverKP = 0.007F;
        public static float revolverKD= 0.00025F;
        public static float revolverKI = 0F;
    }

    @Config
    public static class launcher {
        public static float launcherTol = 5F;
        public static double flykP =0.0F;
        public static double flykI =0.0F;
        public static double flykD = 0.0F ;
        public static double flykF = 0.000197F;

        public static double downset=0.0F;
        public static double upset=270.0F;

    }







}