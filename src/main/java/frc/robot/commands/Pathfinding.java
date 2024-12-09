package frc.robot.Auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.LinkedList;
import java.util.function.BooleanSupplier;

/** Pathfinding */
public class Pathfinding {
  public enum POI {
    Note1(0.0, 0.0, 0.0, () -> ConditionBuilder.intakeHasNote()),
    Note2(0.0, 0.0, 0.0, () -> false);

    private double x_coordinates;
    private double y_coordinates;
    private Rotation2d angle;
    private Translation2d xy_coordinates;
    private BooleanSupplier[] conditions;

    /**
     * constructor stocking all the data we need per poi in variables
     *
     * @param x_coordinates the x position of the poi in (x,y)
     * @param y_coordinates the y position of the poi in (x,y)
     * @param angle the angle the robot should be facing once reaching the poi
     * @param removeCondition the condition to which we remove the poi from a list
     */
    private POI(
        double x_coordinates,
        double y_coordinates,
        double angle,
        BooleanSupplier... removeCondition) {
      this.conditions = removeCondition;
      this.x_coordinates = x_coordinates;
      this.y_coordinates = y_coordinates;
      this.angle = new Rotation2d(angle);
      this.xy_coordinates = new Translation2d(this.x_coordinates, this.y_coordinates);
    }

    /**
     * constructor overwritting the other one to allow input of a Translation2d object
     *
     * @param xy_coordinates the x and y position of the poi in (x,y)
     * @param angle the angel the robot should facing once reaching the poi
     * @param removeCondition the condition to which we remove the poi from a list
     */
    private POI(Translation2d xy_coordinates, double angle, BooleanSupplier... removeCondition) {
      this.conditions = removeCondition;
      this.angle = new Rotation2d(angle);
      this.xy_coordinates = xy_coordinates;
    }

    /**
     * @return the angle we want the robot to face when reaching this point
     */
    private Rotation2d getAngle() {
      return this.angle;
    }

    /**
     * @return the position of the robot when reaching the POI
     */
    private Translation2d getCoordinates() {
      return this.xy_coordinates;
    }

    private boolean getConditionStatus() {
      boolean allCondionsTrue = false;
      byte trueConditions = 0;
      for (BooleanSupplier condition : conditions) {
        if (condition.getAsBoolean() == true) {
          trueConditions += 1;
        }
        if (trueConditions == conditions.length) {
          allCondionsTrue = true;
        }
      }

      return allCondionsTrue;
    }
  }

  private static LinkedList<POI> poiList = new LinkedList<>();

  private static PathConstraints constraints =
      new PathConstraints(3.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720));

  /**
   * generic function to override. If not overriden will always return 0.
   *
   * @param poi_reward the poi to which we want to estimate the reward
   * @return the reward in points per meter or another similar unit which must involve points
   */
  public static double rewardFunction(POI poi_reward) {
    return 0.0;
  }

  /**
   * this methods goal is to filter through all the useful POIs and select the most advantageous one
   *
   * @param poi a list of POIs to filter through
   * @return The {@link Pose2d} of the most useful point
   */
  private static Pose2d FilterPOIs(LinkedList<POI> poi) {
    double maxReward = 0.0;
    // removes POIs which conditions aren't true this also ensures that the robot doesn't perform a
    // useless action
    poi.removeIf(offendingPoint -> (!offendingPoint.getConditionStatus()));
    // loop picking the highest possible reward which should be in points per meter or a similar
    // unit
    for (POI poi_reward : poi) {
      double reward = rewardFunction(poi_reward);
      if (maxReward < reward) {
        maxReward = reward;
        poi.removeFirstOccurrence(poi_reward);
        poi.addFirst(poi_reward);
      }
    }
    return new Pose2d(poi.peekFirst().getCoordinates(), poi.pop().getAngle());
  }

  /**
   * executes the pathfinding command meaning that the robot should go to all chosen POIs
   *
   * @param poi a list of POIs that the robot must go through if condtions apply
   * @return the command to pathfind to a specified point
   */
  public static Command doPathfinding(POI[] poi) {
    if (poiList.isEmpty()) {
      for (POI poiArrayElement : poiList) {
        poiList.add(poiArrayElement);
      }
    }
    return Commands.run(() -> AutoBuilder.pathfindToPose(FilterPOIs(poiList), constraints));
  }
}

// this class could be in constants
final class ConditionBuilder {
  // example code to demonstrate how we could use functions to make our conditions
  public static boolean intakeHasNote() {
    boolean conditionFullfilled = false;
    boolean somecondition = true;
    if (somecondition == true) {
      conditionFullfilled = true;
    }
    return conditionFullfilled;
  }
}
