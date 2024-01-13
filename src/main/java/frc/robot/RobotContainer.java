package frc.robot;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.SwerveSubsystem;

import java.io.File;
import java.util.ArrayList;
import java.util.HashMap;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

public class RobotContainer {

    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

    private final Joystick driverJoytick = new Joystick(OIConstants.kDriverControllerPort);

    SendableChooser<Command> autoChooser = new SendableChooser<>();


    // private final HashMap<String, Command> events = new HashMap<>();
    // private SendableChooser<Command> m_chooser = new SendableChooser<>();
    // private File[] m_autoPathFiles = new File(Filesystem.getDeployDirectory(), "pathplanner/").listFiles();
    //private final SwerveAutoBuilder autoBuilder= new SwerveAutoBuilder(swerveSubsystem::getPose, swerveSubsystem::resetOdometry, new PIDConstants(5, 0, 0), new PIDConstants(5, 0, 0), swerveSubsystem::setModuleStates, events, swerveSubsystem);


    public RobotContainer() {
        //configureAutoChooser();
        
        swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                swerveSubsystem,
                () -> -driverJoytick.getRawAxis(OIConstants.kDriverYAxis),
                () -> driverJoytick.getRawAxis(OIConstants.kDriverXAxis),
                () -> driverJoytick.getRawAxis(OIConstants.kDriverRotAxis),
                () -> !driverJoytick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));

        configureButtonBindings();

        // autoChooser.setDefaultOption("No Auto Selected", new WaitCommand(1.0));
        // autoChooser.addOption("Straight", null);

    }
    private void configureButtonBindings() {
        //new JoystickButton(driverJoytick, 2).whenPressed(() -> swerveSubsystem.zeroHeading());
  }

  public Command getAutonomousCommand(){
    // Load the path you want to follow using its name in the GUI
    return new PathPlannerAuto("New Auto");
    }

  }
