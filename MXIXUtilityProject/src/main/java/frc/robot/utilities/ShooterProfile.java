package frc.robot.utilities;

public class ShooterProfile {
    private final double distanceMeters;
    private final double hoodAngleDegrees;
    private final double shooterVelocityRPM;

    // we can add other things like backspin or whatever later if needed


    public ShooterProfile(double distanceMeters, double hoodAngleDegrees, double shooterVelocityRPM) {
        this.distanceMeters = distanceMeters;
        this.hoodAngleDegrees = hoodAngleDegrees;
        this.shooterVelocityRPM = shooterVelocityRPM;
    }

    public double getDistance() {
        return distanceMeters;
    }

    public double getHoodAngle() {
        return hoodAngleDegrees;
    }

    public double getVelocityRPM() {
        return shooterVelocityRPM;
    }
}
