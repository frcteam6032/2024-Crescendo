
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

public class IntakeSubsystem_2023 extends SubsystemBase {

private PWMSparkMax motorController1;
private PWMSparkMax motorController2;

    public IntakeSubsystem_2023() {
 
motorController1 = new PWMSparkMax(0);
 addChild("Motor Controller 1",motorController1);
 motorController1.setInverted(false); // Keeping left motor rotation the same

motorController2 = new PWMSparkMax(1);
 addChild("Motor Controller 2",motorController2);
 motorController2.setInverted(true); // Inverting right motor rotation to intake objects

    }

    public void set_speed(double value){
        motorController1.set(value);
        motorController2.set(value);
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

