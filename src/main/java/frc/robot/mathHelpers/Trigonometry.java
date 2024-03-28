package frc.robot.mathHelpers;

public class Trigonometry {
    public static double calcuateAngle(double distanceX, double distanceY) {
        return Math.atan2(distanceX, distanceY);
    }

    public static double offsetByAngle(double angle, double offset) {
        return angle + offset;
    }

    public static double calculateAllowedDeviationAngle(double angle) {
        return (angle * 0.5) + angle;
    }

    public static double calculateAllowedDeviationDistance(double distance) {
        return (distance * 0.5) + distance;
    }
}
