package org.firstinspires.ftc.teamcode.hardware;


import com.acmerobotics.dashboard.config.Config;

@Config
public class Globals {
    public static float fwKa = 0.01F; //basically does nothing
    public static float fwKs = 0.085F; //small change
    public static float fwKv = 0.00014F; //main change

    public static float turretKP = 0.020F;
    public static float turretKI = 0.000F;
    public static float turretKD = 0.002F;
    public static float turretKF = 0.000F;

    public static float turretTol = 0.05F;
    public static float turretMin = 0.2F;


    public static float targetrpm = 6000F;

    public static double downset=0.0F;
    public static double upset=130F;

    public static double kPturn = 0.6F;        // scale turn power by angle error
    public static double minPower = 0.08;
    public static double maxPower = 1.0;
    public static double alignDegTol = 1.5;   // done when |bearing| < this
    public static double minDecisionMargin = 15.0; // ignore sketchy detections

}