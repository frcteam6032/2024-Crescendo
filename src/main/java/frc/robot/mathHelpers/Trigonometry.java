package frc.robot.mathHelpers;

public class Trigonometry {
    // Convert Cartesian coordinates to spherical coordinates
    public static double[] cartesianToSpherical(double x, double y, double z) {
        double range = Math.sqrt(x * x + y * y + z * z);
        double azimuth = Math.atan2(y, x); // Returns value in radians
        double elevation = Math.atan2(z, Math.sqrt(x * x + y * y));
        return new double[] { azimuth, elevation, range };
    }

    public static double distanceX(double angleDegrees, double distanceY) {
        double angleRadians = Math.toRadians(angleDegrees);
        double distance = Math.tan(angleRadians) * distanceY;
        return distance;
    }
}
