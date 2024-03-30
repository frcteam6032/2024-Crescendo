package frc.robot.mathHelpers;

public class Trigonometry {
    // Convert normal coordinates to spherical coordinates to acccount for arc
    // curvature
    static public double[] cartesianToSpherical(double x, double y, double z) {
        // Multiply variables by themselves and not use Math.pow for efficiency
        double range = Math.sqrt(x * x + y * y + z * z);
        // This is the yaw in spherical coordinates. Use the arc tangent of y and x to get the angle from the center of the april tag to the robot
        double azimuth = Math.atan2(y, x);
        double elevation = Math.atan2(z, Math.sqrt(x * x + y * y));
        return new double[] { azimuth, elevation, range };
    }
}
