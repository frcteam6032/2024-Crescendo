package frc.robot.auto;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.*;
public class autoBuilder {
    public static Command autoCommandBuilder(int autoCommand) {
        Command returnCommand = null;
            switch (autoCommand) {
                case 1:
                    // Drive forward
                    break;
                case 2:
                    // Drive backward
                    break;
                case 3:
                    // Turn left
                    break;
                case 4:
                    // Turn right
                    break;
                default:
                    // Do nothing
                    break;
            }
            return returnCommand;
    }
}
