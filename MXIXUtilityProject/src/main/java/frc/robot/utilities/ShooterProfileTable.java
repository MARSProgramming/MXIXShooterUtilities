package frc.robot.utilities;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShooterProfileTable {

    private final InterpolatingDoubleTreeMap hoodMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap velocityMap = new InterpolatingDoubleTreeMap();

    /**
     * Add a measured shooter profile to the table.
     * Distance is the independent variable.
     */
    public void addProfile(ShooterProfile profile) {
        hoodMap.put(profile.getDistance(), profile.getHoodAngle());
        velocityMap.put(profile.getDistance(), profile.getVelocityRPM());
    }

    /**
     * Returns an interpolated ShooterProfile for the given distance.
     */
    public ShooterProfile getProfile(double distanceMeters) {
        return new ShooterProfile(
            distanceMeters,
            hoodMap.get(distanceMeters),
            velocityMap.get(distanceMeters)
        );
    }
}
