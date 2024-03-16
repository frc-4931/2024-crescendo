package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class DoubleMotor extends SubsystemBase {
    private boolean isOn;
    private double speed1 = 0.7;
    private double speed2 = -0.7;
    private String subsystem;
    private final CANSparkMax motor1;
    private final CANSparkMax motor2;
    
    /**
     * Constructer
     * starts off
     */
    public DoubleMotor(String subsystem, double speed1, double speed2, int motorId1, int motorId2) {
        this.subsystem = subsystem;
        this.speed1 = speed1;
        this.speed2 = speed2;
        isOn = false;
        motor1 = new CANSparkMax(motorId1 , MotorType.kBrushless);
        motor2 = new CANSparkMax(motorId2, MotorType.kBrushless);
        motor1.setInverted(false);
        motor2.setInverted(false);
        SmartDashboard.putNumber(subsystem + "TargetSpeed1", speed1);
        SmartDashboard.putNumber(subsystem + "TargetSpeed2", speed2);
    }

    public Command toggle() {
        return this.runOnce(() -> {
            isOn = !isOn;
            setSpeed(isOn ? speed1 : 0, isOn ? speed2 : 0);
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
        setSpeed(speed1, speed2);
    }
    
    public void turnOff() {
        isOn = false;
        setSpeed(0,0);
    }

    public void reverse() {
        setSpeed(-speed1, -speed2);
    }

    /**
     * Speed Should NEVER exceed the domain (-1,1)
     * @param speed1,speed2
     */
    private void setSpeed(double speed1, double speed2) {
        motor1.set(MathUtil.clamp(speed1, -1, 1));
        motor2.set(MathUtil.clamp(speed2, -1, 1));
        SmartDashboard.putBoolean(subsystem, isOn);
    }

    @Override
    public void periodic() {
        double s = SmartDashboard.getNumber(subsystem + "TargetSpeed1", speed1);
        if (s != speed1) {
            speed1 = s;
        }
        double b = SmartDashboard.getNumber(subsystem + "TargetSpeed2", speed2);
        if (b != speed2) {
            speed2 = b;
        }
    }
}
