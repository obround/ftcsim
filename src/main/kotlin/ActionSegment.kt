import com.acmerobotics.roadrunner.geometry.Pose2d
import trajectorysequence.sequencesegment.SequenceSegment


/**
 * The type of non-movement action performed by the robot
 *
 * @param duration: The time it takes to perform the certain action
 * @param distanceDropped: The distance in front of the robot the pixel is dropped in inches;
 *                         Only applicable to DROP_PIXEL
 * TODO: Find the actual duration and distance dropped values
 **/
enum class Action(val duration: Double, val distanceDropped: Double?) {
    DROP_PIXEL(3.0, 2.5),
    DROP_PIXEL_ON_BOARD(4.9, null)
}

class ActionSegment(val action: Action, pose: Pose2d) : SequenceSegment(action.duration, pose, pose, emptyList())
