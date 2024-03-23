package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AutoCommand;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.DoubleMotor;
//import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.Sensors;
import frc.robot.subsystems.Shelf;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Vision;

public class RobotContainer {
   //private final Vision vision = new Vision();
    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    
    // private final PoseEstimator poseEstimator = new PoseEstimator(swerveSubsystem, vision,
    //         new Pose2d(2, 7, swerveSubsystem.getRotation2d()));
    // private AutoCommand autoComands = new AutoCommand(vision, swerveSubsystem);
    private final CommandXboxController driverJoystick = new CommandXboxController(OIConstants.kDriverControllerPort);
    private final CommandJoystick buttonBox = new CommandJoystick(1);
    //1 is the top, 2 is the bottom
    private DoubleMotor intakeUse = new DoubleMotor("intake", 0.4, 0.6, 9, 14);
    private DoubleMotor conveyer = new DoubleMotor("conveyer", -0.6, 0.6, 12, 13);
    private Shooter shooterUse = new Shooter();
    private Shelf shelfUse= new Shelf();
    private Sensors sensors = new Sensors();

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        // CameraServer.startAutomaticCapture();
        swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                swerveSubsystem,
                () -> -driverJoystick.getRawAxis(OIConstants.kDriverYAxis),
                () -> -driverJoystick.getRawAxis(OIConstants.kDriverXAxis),
                () -> -driverJoystick.getRawAxis(OIConstants.kDriverRotAxis),
                () -> !driverJoystick.b().getAsBoolean()));

        // NamedCommands.registerCommand("PathPlan", autoComands.toNote());
        NamedCommands.registerCommand("Shoot", shooterUse.toggleFast().andThen(Commands.waitSeconds(0.6)).andThen(conveyer.turnOn()));
        NamedCommands.registerCommand("intake", intakeUse.turnOn().andThen(conveyer.turnOn()));
        NamedCommands.registerCommand("off", shooterUse.stop().andThen(conveyer.turnOff()).andThen(intakeUse.turnOff()));

        configureButtonBindings();

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto", autoChooser);
    }

    private void configureButtonBindings() {
        // new JoystickButton(buttonBox, 1).onTrue(autoComands.toNote());

        // PathPlannerPath spin = PathPlannerPath.fromPathFile("Spin");
        driverJoystick.leftBumper().onTrue(swerveSubsystem.zeroHeadingCommand());
        driverJoystick.rightBumper().onTrue(shelfUse.closeAntenna().andThen(shooterUse.stop()).andThen(conveyer.turnOff())); //all down
        //driverJoystick.rightTrigger().onTrue(shooterUse.toggleFast());
        // driverJoystick.rightStick().onTrue(shelfUse.toggleClimbers());
        driverJoystick.a().onTrue( //intakeUse.turnOn().andThen(conveyer.turnOn()));
        new FunctionalCommand(() -> { intakeUse.on(); conveyer.on(); }, () -> {},
         (interrupted) -> { intakeUse.off(); conveyer.off(); },
         sensors.getDio(), intakeUse, conveyer, sensors)
        );
        // .until(sensors.getDio()).finallyDo(() -> {

        //     
        // })); //intake
        //driverJoystick.b().onTrue(conveyer.toggle()); 
         driverJoystick.y().onTrue(shelfUse.toggleShelf()); //shelf up
         driverJoystick.x().onTrue(//conveyer -> amp
         shooterUse.runSlow().andThen(shelfUse.openAntenna()).andThen(shelfUse.closeShelf()).andThen(Commands.waitSeconds(0.6)).andThen(conveyer.turnOn())); //pin up
         driverJoystick.leftTrigger().onTrue(shelfUse.toggleShelf());
         driverJoystick.povDown().onTrue(intakeUse.turnOff());
         driverJoystick.povUp().onTrue(shooterUse.stop().andThen(conveyer.turnOff()));
         driverJoystick.povLeft().onTrue(shelfUse.openShelf());
         driverJoystick.rightStick().onTrue(shelfUse.toggleClimbers());
        //  driverJoystick.povRight().onTrue();



        //Button Box
         buttonBox.button(1).onTrue(intakeUse.toggleReverse());
         buttonBox.button(2).onTrue(intakeUse.turnOff());
         buttonBox.button(3).onTrue(conveyer.toggleReverse());
         buttonBox.button(4).onTrue(conveyer.turnOff());
         buttonBox.button(5).onTrue(shooterUse.runReverse());
         buttonBox.button(6).onTrue(shooterUse.stop());

         //lowered wait from .5 becuase we wait too long
         buttonBox.button(7).onTrue(shooterUse.toggleFast().andThen(Commands.waitSeconds(.3)).andThen(conveyer.turnOn()));
         //give me shooter

         buttonBox.button(9).onTrue(shelfUse.openClimber());
         buttonBox.button(10).onTrue(shelfUse.closeClimber());
         


        //FIXME: need to get a stop from the throughbeam
        // DigitalInput diThroughBeam = new DigitalInput(0);
        // Command autoOff = new FunctionalCommand(() -> { intakeUse.turnOn(); conveyer.turnOn(); },
        //     () -> {},
        //     interrupted -> { intakeUse.turnOff(); conveyer.turnOff(); },
        //     () -> diThroughBeam.get(),
        //     intakeUse, conveyer
        // );
        // SmartDashboard.putBoolean("Through Beam", diThroughBeam.get());


        
        
        // driverJoytick.rightBumper().onTrue(AutoBuilder.followPath(spin));
        // buttonBox.button(1).onTrue(intakeUse.toggle(0.7));
        buttonBox.button(2).onTrue(shooterUse.toggleFast());
        buttonBox.button(3).onTrue(shooterUse.toggleSlow());
    }
    // new JoystickButton(buttonBox, 4).onTrue(autoComands.PathToPose(1, 1, 0));

    // // //cool spin move
    // new JoystickButton(buttonBox, 2).onTrue(Commands.runOnce(() -> {
    // //get current pose
    // Pose2d currentPose = swerveSubsystem.getPose();

    // //make start, mid, and end pose
    // Pose2d startPos = new Pose2d(currentPose.getTranslation(), new Rotation2d());
    // Pose2d midPos = new Pose2d(currentPose.getTranslation().plus(new
    // Translation2d(1.0, 0.5)), new Rotation2d());
    // Pose2d endPos = new Pose2d(currentPose.getTranslation().plus(new
    // Translation2d(2.0, 0.0)), new Rotation2d());

    // List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(startPos,
    // midPos, endPos);
    // PathPlannerPath path = new PathPlannerPath(
    // bezierPoints,
    // new PathConstraints(
    // 3.81, 2.0,
    // Units.degreesToRadians(360), Units.degreesToRadians(540)
    // ),
    // new GoalEndState(0.0, Rotation2d.fromDegrees(0))
    // );

    // // Prevent this path from being flipped on the red alliance, since the given
    // positions are already correct
    // path.preventFlipping = true;

    // AutoBuilder.followPath(path).schedule();
    // }));

    public Command getAutonomousCommand() {
        swerveSubsystem.zeroHeading();
        return autoChooser.getSelected();

    }
}