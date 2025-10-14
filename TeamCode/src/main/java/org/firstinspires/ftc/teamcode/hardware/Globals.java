package org.firstinspires.ftc.teamcode.hardware;


import com.acmerobotics.dashboard.config.Config;

@Config
public class Globals {
    public static float fwKa = 0.01F; //basically does nothing
    public static float fwKs = 0.2F; //small change
    public static float fwKv = 0.00014F; //main change
    public static float launcherTol = 100F;

    public static float turretKP = 0.018F;
    public static float turretKI = 0.000F;
    public static float turretKD = 0.013F;
    public static float turretKF = 0.000F;

    public static float turretTol = 2F;
    public static float turretMin = 0.11F;
    public static float turretMax = 0.8F;
    public static float turretCamOffset = 5.0F;

    public static int oneRotation = 753; // actually 2 rotations lol  || Minus is counter-clockwise rotation.
    public static int revolverTol = 0;
    public static float revolverKP = 0.006F;
    public static float revolverKD=0.002F;
    public static float targetrpm = 6000F;

    public static double downset=0.0F;
    public static double upset=170F;

    public static double kPturn = 0.6F;        // scale turn power by angle error
    public static double minPower = 0.08;
    public static double maxPower = 1.0;
    public static double alignDegTol = 1.5;   // done when |bearing| < this
    public static double minDecisionMargin = 15.0; // ignore sketchy detections
    public static double cpr=384.5;
    public static double rpm=435.0;
}