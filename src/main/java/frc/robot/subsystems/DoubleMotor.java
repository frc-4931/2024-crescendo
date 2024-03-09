package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class DoubleMotor extends SubsystemBase {
    private boolean isOn;
    private double speed = 0.7;
    private String subsystem;
    private final CANSparkMax motor1;
    private final CANSparkMax motor2;
    
    /**
     * Constructer
     * starts off
     */
    public DoubleMotor(String subsystem, double speed, int motorId1, int motorId2) {
        this.subsystem = subsystem;
        this.speed = speed;
        isOn = false;
        motor1 = new CANSparkMax(motorId1 , MotorType.kBrushless);
        motor2 = new CANSparkMax(motorId2, MotorType.kBrushless);
        motor2.setInverted(true);
        motor2.follow(motor1);
        SmartDashboard.putNumber(subsystem + "TargetSpeed", speed);
    }

    public Command toggle() {
        return this.runOnce(() -> {
            isOn = !isOn;
            setSpeed(isOn ? speed : 0);
        });
    }

    public Command on() {
        return this.run(this::turnOn);
    }

    public Command off() {
        return this.run(this::turnOff);
    }

    public void turnOn() {
        isOn = true;
        setSpeed(speed);
    }
    
    public void turnOff() {
        isOn = false;
        setSpeed(0);
    }

    public void reverse() {
        setSpeed(-speed);
    }

    /**
     * Speed Should NEVER exceed the domain (-1,1)
     * @param speed
     */
    private void setSpeed(double speed) {
        motor1.set(MathUtil.clamp(speed, -1, 1));
        SmartDashboard.putBoolean(subsystem, isOn);
    }

    @Override
    public void periodic() {
        double s = SmartDashboard.getNumber(subsystem + "TargetSpeed", speed);
        if (s != speed) {
            speed = s;
        }
    }
}
