package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Intake extends SubsystemBase {
    private boolean isOn;
    private final CANSparkMax intakeMotor;
    private final CANSparkMax intakeMotor2;
    private static final CANSparkLowLevel.MotorType kMotorType = CANSparkLowLevel.MotorType.kBrushless;
    /**
     * Constructer
     * starts off
     */
    public Intake() {
        isOn = false;
        intakeMotor = new CANSparkMax(9 , kMotorType);
        intakeMotor2 = new CANSparkMax(14, kMotorType);
    }
    public Command toggle(double speed) {
        return this.runOnce(() -> {isOn = !(isOn); setSpeed(speed);});
    }
    public void turnOn(double speed) {
        isOn = true;
        setSpeed(speed);
    }
    
    public void turnOff(double speed) {
        isOn = false;
        setSpeed(0);
    }
    /**
     * Speed Should NEVER exceed the domain (-1,1)
     * @param speed
     */
    private void setSpeed(double speed) {
        if(isOn) {
              intakeMotor.set(speed);
              intakeMotor2.set(speed*-1);
        }
        else {
        intakeMotor.set(0);
        intakeMotor2.set(0);}


    SmartDashboard.putBoolean("Intake", isOn);
    }
}
