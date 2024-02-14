package frc.robot.mathHelpers;

public class trig {
    public static double calcuateAngle(double distanceX, double distanceY) {
        return Math.atan2(distanceY, distanceX);
    }
}
