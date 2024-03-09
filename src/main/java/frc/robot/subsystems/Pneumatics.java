package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pneumatics extends SubsystemBase {
    private Compressor compressor;


    @Override
    public void periodic() {
        SmartDashboard.putNumber("air - analog voltage", compressor.getAnalogVoltage());
        SmartDashboard.putNumber("air - pressure", compressor.getPressure());
    }
    
}
