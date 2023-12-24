import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.trajectory.constraints.*
import javafx.animation.KeyFrame
import javafx.animation.Timeline
import javafx.application.Application
import javafx.scene.Scene
import javafx.scene.canvas.Canvas
import javafx.scene.canvas.GraphicsContext
import javafx.scene.image.Image
import javafx.scene.layout.Pane
import javafx.scene.transform.Rotate
import javafx.stage.Stage
import javafx.util.Duration
import trajectorysequence.TrajectorySequence
import trajectorysequence.TrajectorySequenceBuilder
import trajectorysequence.sequencesegment.TrajectorySegment
import trajectorysequence.sequencesegment.TurnSegment
import trajectorysequence.sequencesegment.WaitSegment


class Simulator : Application() {
    // CONSTANTS
    private val fps = 30
    private val startPose = Pose2d(0.0, -24.0 * 3, 0.0)
    private val fieldDimPixels = 640  // 640px x 640px
    private val fieldDimReal = 144  // 144in x 144in
    private val robotWidth = 17.995.in2pix  // 17.995in (rear to intake tip)
    private val robotLength = 17.319.in2pix // 17.319in (side to side)
    private val trackWidth = 12.7  // distance between wheels
    private val maxVel = 60.0
    private val maxAccel = 60.0
    private val maxAngVel = 5.528055275836724
    private val maxAngAccel = 5.528055275836724

    // OTHER
    private val field = Image("/field.png")
    private val robot = Image("/robot_ftc.png", robotWidth, robotLength, true, true)
    private var startTime = Double.NaN
    private var sequence = getTrajectorySequence()
    private var segmentIndex = 0
    private lateinit var timeline: Timeline
    private var trajectoryDurations = sequence.sequenceList.map { it.duration }

    private val Double.in2pix get() = this * fieldDimPixels / fieldDimReal
    private val Pose2d.in2pix get() = Pose2d(this.x.in2pix, this.y.in2pix, heading.toDegrees)

    private val windowWidth = field.width
    private val windowHeight = field.height

    override fun start(stage: Stage) {
        val root = Pane()
        val canvas = Canvas(field.width, field.height)

        stage.title = "FTCSIM"
        stage.isResizable = false

        timeline = Timeline(KeyFrame(
            Duration.millis(1000.0 / fps),
            { runSimulation(canvas.graphicsContext2D) }
        ))
        timeline.cycleCount = Timeline.INDEFINITE

        root.children.add(canvas)
        stage.scene = Scene(root, windowWidth, windowHeight)
        stage.show()
        timeline.play()
    }

    private fun runSimulation(gc: GraphicsContext) {
        gc.drawImage(field, 0.0, 0.0)
        when (val segment = sequence.get(segmentIndex)) {
            is TrajectorySegment -> updateRobotMove(gc, segment)
            is TurnSegment -> updateRobotTurn(gc, segment)
            is WaitSegment -> TODO()
        }

        if (segmentIndex == sequence.size()) timeline.stop()
    }

    private fun updateRobotMove(gc: GraphicsContext, segment: TrajectorySegment) {
        val trajectory = segment.trajectory
        val profileTime = getProfileTime()

        if (profileTime >= trajectory.duration()) segmentIndex++

        val (x, y, heading) = trajectory[profileTime].in2pix
        gc.save()
        gc.rotate(heading)
        gc.drawImage(robot, x, -y)
        gc.restore()
    }

    private fun updateRobotTurn(gc: GraphicsContext, segment: TurnSegment) {
        val motion = segment.motionProfile
        val profileTime = getProfileTime()

        if (profileTime >= segment.duration) segmentIndex++

        val angle = motion[profileTime].x.toDegrees
        val (x, y, _) = segment.startPose.in2pix
        gc.save()
        // Rotation about pivot point
        val r = Rotate(angle, x + robot.width / 2, -y + robot.height / 2)
        gc.setTransform(r.mxx, r.myx, r.mxy, r.myy, r.tx, r.ty)
        gc.drawImage(robot, x, -y)
        gc.restore()
    }

    private fun getTrajectorySequence(): TrajectorySequence {
        val velocityConstraint = MinVelocityConstraint(
            listOf(
                AngularVelocityConstraint(maxAngVel),
                MecanumVelocityConstraint(
                    maxVel,
                    trackWidth
                )
            )
        )
        val accelerationConstraint = ProfileAccelerationConstraint(maxAccel)
        val builder = TrajectorySequenceBuilder(
            startPose,
            velocityConstraint,
            accelerationConstraint,
            maxAngVel,
            maxAngAccel
        )
        return builder.forward(23.625)
            .strafeRight(23.625)
            .turn((-90.0).toRadians)
            .build()
    }

    private fun currentTime() = System.currentTimeMillis() / 1000.0

    private fun getProfileTime(): Double {
        if (startTime.isNaN()) startTime = currentTime()
        val prevTrajectoryDuration = trajectoryDurations.subList(0, segmentIndex).sum()

        val time = currentTime()
        return time - startTime - prevTrajectoryDuration
    }

}

val Double.toRadians get() = Math.toRadians(this)
val Double.toDegrees get() = Math.toDegrees(this)

fun main(args: Array<String>) {
    Application.launch(Simulator::class.java, *args)
}
