package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Sensors extends SubsystemBase {

    private DigitalInput dio = new DigitalInput(0);

    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        SmartDashboard.putBoolean("throughBeam", dio.get());
    }

    public BooleanSupplier getDio() {
        // return () -> !dio.get();
        return dio::get;
    }
    
}
