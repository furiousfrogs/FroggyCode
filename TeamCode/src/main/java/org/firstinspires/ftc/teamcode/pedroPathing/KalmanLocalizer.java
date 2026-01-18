package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.localization.Localizer;
import com.pedropathing.geometry.Pose; // make sure this import matches Pedro's Pose type
import com.pedropathing.math.Vector;

public class KalmanLocalizer implements Localizer {

    private Pose pose = new Pose(0, 0, 0);
    @Override
    public Pose getPose() {
        return pose;
    }

    @Override
    public Pose getVelocity() {
        return null;
    }

    @Override
    public Vector getVelocityVector() {
        return null;
    }

    @Override
    public void setStartPose(Pose setStart) {

    }

    @Override
    public void setPose(Pose pose) {
        this.pose = pose;
    }
    @Override
    public void update() {
        // TODO: sensor read + kalman update
    }

    @Override
    public double getTotalHeading() {
        return 0;
    }

    @Override
    public double getForwardMultiplier() {
        return 0;
    }

    @Override
    public double getLateralMultiplier() {
        return 0;
    }

    @Override
    public double getTurningMultiplier() {
        return 0;
    }

    @Override
    public void resetIMU() throws InterruptedException {

    }

    @Override
    public double getIMUHeading() {
        return 0;
    }

    @Override
    public boolean isNAN() {
        return false;
    }
}
