package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

// import frc.robot.subsystems.Elevator;
// import frc.robot.subsystems.LEDs;
// import frc.robot.subsystems.swerve.Swerve;

public class SpeakerLock extends Command {
  private PhotonCamera m_camera;
  // private Swerve m_swerve;
  // private Elevator m_elevator;
  private DoubleSupplier m_translationSup;
  private DoubleSupplier m_strafeSup;
  // private LEDs m_led;
  private int m_alliance_index;

  /**
   * Command to keep the aiming at the speaker while keeping the robot in motion
   *
   * @param s_swerve swerve submodule instance
   * @param s_led led submodule instance
   * @param translationSup translation forward or backward
   * @param strafeSup moving laterally
   * @param camera photon camera instance
   */
  public SpeakerLock(
      // Swerve s_swerve,
      // LEDs s_led,
      DoubleSupplier translationSup, DoubleSupplier strafeSup, PhotonCamera camera) {
    this.m_camera = camera;

    // this.m_swerve = s_swerve;
    // this.m_led = s_led;

    // find out the current alliance with fail safe
    m_alliance_index = 0;
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) m_alliance_index = alliance.get() == Alliance.Red ? 0 : 1;

    // addRequirements(s_swerve);
    // addRequirements(s_led);

    this.m_translationSup = translationSup;
    this.m_strafeSup = strafeSup;
  }

  @Override
  public void initialize() {}

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    // m_swerve.drive(new Translation2d(0, 0), 0, false, true);
    // m_elevator.extendTheElevator(Elevator.elevatorHeight.LOW);
    // m_led.setState(LEDs.State.IDLE);
  }

  @Override
  public void execute() {
    double translationVal =
        MathUtil.applyDeadband(m_translationSup.getAsDouble(), Constants.stickDeadband);

    double strafeVal = MathUtil.applyDeadband(m_strafeSup.getAsDouble(), Constants.stickDeadband);
    double rotationVal = 0;
    // double rotationVal = m_swerve.getRotation2d().getRadians();

    PhotonTrackedTarget target = null;

    // Vision-alignment mode
    // Query the latest result from PhotonVision
    var result = m_camera.getLatestResult();

    if (result.hasTargets()) {
      for (var t : result.getTargets()) {
        if (t.getFiducialId() == Constants.VisionConstants.kSpeakerIndex[m_alliance_index]) {
          target = t;
          break;
        }
      }
    }

    // still possible that the visible ids are not the ones we're interested in
    if (target != null) {
      // TODO this will most likely require some sort of conversion to speed
      rotationVal = target.getYaw();
    }

    /* Rotate to face speaker */
    // m_swerve.drive(
    // new Translation2d(translationVal,
    // strafeVal).times(Constants.Swerve.maxSpeed),
    // m_rotDirection * 0.2 * Constants.Swerve.maxAngularVelocity, // constant speed
    // false,
    // true);

  }
}
