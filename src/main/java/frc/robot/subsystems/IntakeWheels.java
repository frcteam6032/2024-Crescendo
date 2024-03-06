
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

public class IntakeWheels extends SubsystemBase {

private PWMSparkMax motorController1;

    public IntakeWheels() {
 
    motorController1 = new PWMSparkMax(4);
    addChild("Motor Controller 4",motorController1);
    motorController1.setInverted(false); // Keeping left motor rotation the same



    }

    public void set_speed(double value){
        motorController1.set(value);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

}