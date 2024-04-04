package frc.robot.mathHelpers;

public class Trigonometry {
    // Convert normal coordinates to spherical coordinates to acccount for arc
    // curvature
    static public double[] cartesianToSpherical(double x, double y, double z) {
        // Multiply variables by themselves and not use Math.pow for efficiency
        double range = Math.sqrt(x * x + y * y + z * z);
        // This is the yaw in spherical coordinates. Use the arc tangent of y and x to get the angle from the center of the april tag to the robot
        // Arc-tangent is the inverse of tangent and takes in 2 side lengths and returns the angle
        double azimuth = Math.atan2(y, x);
        double elevation = Math.atan2(z, Math.sqrt(x * x + y * y));
        return new double[] { azimuth, elevation, range };
    }

    static public double distanceX(double angle, double distanceY) {
        // SOH CAH TOA
        // We need to find the bottom side length given the distance for the other side
        // So we are given the topmost angle
        // We can use the tangent function to find the bottom side length
        // tan(angle) = opposite / adjacent

        // We are solving for one variable 
        double distance = 0;
        distance = Math.tan(angle) * distanceY;

        return distance;
    }
}
