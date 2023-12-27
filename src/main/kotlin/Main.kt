import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint
import javafx.animation.KeyFrame
import javafx.animation.Timeline
import javafx.application.Application
import javafx.geometry.Rectangle2D
import javafx.geometry.VPos
import javafx.scene.Scene
import javafx.scene.canvas.Canvas
import javafx.scene.canvas.GraphicsContext
import javafx.scene.control.Alert
import javafx.scene.control.ButtonType
import javafx.scene.image.Image
import javafx.scene.layout.Pane
import javafx.scene.paint.Color
import javafx.scene.text.Font
import javafx.scene.text.TextAlignment
import javafx.stage.Stage
import javafx.util.Duration
import trajectorysequence.TrajectorySequence
import trajectorysequence.TrajectorySequenceBuilder
import trajectorysequence.TrajectoryType
import trajectorysequence.sequencesegment.SequenceSegment
import trajectorysequence.sequencesegment.TrajectorySegment
import trajectorysequence.sequencesegment.TurnSegment
import trajectorysequence.sequencesegment.WaitSegment
import kotlin.math.cos
import kotlin.math.sin


class Simulator : Application() {
    // CONSTANTS
    private val fps = 60  // Frames per second
    private val numberSamples = 50  // Number of samples when drawing the trajectory
    private val startPose = Pose2d(0.0, -24.0 * 4 - 6.681 + 2, 180.0.toRadians)  // x, y, angle
    private val fieldDimPixels = 640.0  // 640px x 640px
    private val fieldDimReal = 144.0  // 144in x 144in
    private val robotWidth = 17.995  // 17.995in (rear to intake tip)
    private val robotLength = 17.319 // 17.319in (side to side)
    private val trackWidth = 12.7  // Distance between wheels
    private val maxVel = 60.0  // mph
    private val maxAccel = 60.0  // mph
    private val maxAngVel = 5.528055275836724  // rad/sec
    private val maxAngAccel = 5.528055275836724  // rad/sec^2

    // COLLISION
    private val leftBackdrop = Rectangle2D(105.0, 0.0, 110.0, 55.0)
    private val rightBackdrop = Rectangle2D(425.0, 0.0, 110.0, 55.0)
    private val trussLeg1 = Rectangle2D(0.0, 321.0, 3.0, 105.0)
    private val trussLeg2 = Rectangle2D(105.0, 321.0, 3.0, 105.0)
    private val trussLeg3 = Rectangle2D(212.0, 321.0, 3.0, 105.0)
    private val trussLeg4 = Rectangle2D(425.0, 321.0, 3.0, 105.0)
    private val trussLeg5 = Rectangle2D(637.0, 321.0, 3.0, 105.0)
    private val fixedObjects = arrayOf(
        leftBackdrop,
        rightBackdrop,
        trussLeg1,
        trussLeg2,
        trussLeg3,
        trussLeg4,
        trussLeg5
    )

    // IMAGES
    private val field = Image("/field.png")
    private val pixel = Image("/pixel_ftc.png")
    private val robot = Robot(
        Image("/robot_ftc.png", robotWidth.in2px, robotLength.in2px, true, false),
        fieldDimPixels
    )

    // OTHER
    private var leftBackDropScore = 0
    private var rightBackDropScore = 0
    private val pixelPositions = mutableListOf<Pair<Double, Double>>()
    private var prevPose = startPose
    private val velocityConstraint = MinVelocityConstraint(
        listOf(
            AngularVelocityConstraint(maxAngVel),
            MecanumVelocityConstraint(
                maxVel,
                trackWidth
            )
        )
    )
    private val accelerationConstraint = ProfileAccelerationConstraint(maxAccel)
    private var startTime = Double.NaN
    private var sequence = getTrajectorySequence()
    private var segmentIndex = 0
    private lateinit var timeline: Timeline
    private var trajectoryDurations = sequence.sequenceList.map { it.duration }
    private val collisionAlert = Alert(
        Alert.AlertType.ERROR,
        "ur skill issue got the opps ded crying",
        ButtonType.OK
    )

    // CONVERSIONS
    private val Double.in2px get() = this * fieldDimPixels / fieldDimReal
    private val Pose2d.in2px get() = Pose2d(this.x.in2px, this.y.in2px, heading.toDegrees)

    private val windowWidth = field.width
    private val windowHeight = field.height

    override fun start(stage: Stage) {
        val root = Pane()
        val canvas = Canvas(field.width, field.height)

        collisionAlert.title = null
        collisionAlert.headerText = "Invalid collision with fixed object"

        stage.title = "FTCSIM"
        stage.isResizable = false
        canvas.graphicsContext2D.isImageSmoothing = false

        timeline = Timeline(KeyFrame(
            Duration.millis(1000.0 / fps),
            { simulationLoop(canvas.graphicsContext2D) }
        ))
        timeline.cycleCount = Timeline.INDEFINITE

        root.children.add(canvas)
        stage.scene = Scene(root, windowWidth, windowHeight)
        stage.show()
        timeline.play()
    }

    private fun simulationLoop(gc: GraphicsContext) {
        when (val segment = sequence.get(segmentIndex)) {
            is TrajectorySegment -> updateRobotMove(segment)
            is TurnSegment -> updateRobotTurn(segment)
            is ActionSegment -> updateRobotAction(segment)
            is WaitSegment -> TODO()
        }
        // Render everything
        gc.drawImage(field, 0.0, 0.0)
        robot.render(gc)
        // Draw trajectory
        sequence.sequenceList.forEach { drawTrajectory(gc, it) }
        gc.textAlign = TextAlignment.CENTER
        gc.textBaseline = VPos.CENTER
        gc.font = Font("Arial Bold", 30.0)
        gc.fill = Color.WHITE
        gc.fillText(
            leftBackDropScore.toString(),
            leftBackdrop.centerPointX,
            leftBackdrop.centerPointY - 7
        )
        gc.fillText(
            rightBackDropScore.toString(),
            rightBackdrop.centerPointX,
            rightBackdrop.centerPointY - 7
        )
        pixelPositions.forEach { (x, y) -> gc.drawImage(pixel, x - pixel.width / 2, y - pixel.height / 2) }
        handleCollisions()
        // Stop the animation once we finish the final segment
        if (segmentIndex == sequence.size()) timeline.stop()
    }

    private fun handleCollisions() {
        fixedObjects.forEach { obj ->
            if (robot.collision(obj)) {
                timeline.stop()
                collisionAlert.show()
            }
        }
    }

    private fun updateRobotMove(segment: TrajectorySegment) {
        val trajectory = segment.trajectory

        val profileTime = getProfileTime()
        if (profileTime >= segment.duration) segmentIndex++

        val (x, y, angle) = trajectory[profileTime].in2px
        robot.x = x
        robot.y = -y
        robot.rotation = angle
    }

    private fun updateRobotTurn(segment: TurnSegment) {
        val motion = segment.motionProfile
        val profileTime = getProfileTime()

        if (profileTime >= segment.duration) segmentIndex++

        val angle = motion[profileTime].x
        val (x, y, _) = segment.startPose.in2px
        robot.x = x
        robot.y = -y
        robot.rotation = angle.toDegrees
    }

    private fun updateRobotAction(segment: ActionSegment) {
        when (val action = segment.action) {
            Action.DROP_PIXEL -> {
                if (getProfileTime() >= segment.duration) {
                    val rect = robot.asRectangle2D
                    val rotation = robot.rotation.toRadians
                    pixelPositions.add(
                        Pair(
                            rect.centerPointX + (action.distanceDropped!!.in2px + rect.height / 2) * cos(rotation),
                            rect.centerPointY - (action.distanceDropped.in2px + rect.height / 2) * sin(rotation)
                        )
                    )
                }
            }

            Action.DROP_PIXEL_ON_BOARD ->
                if (robot.distanceTo(leftBackdrop) < robot.distanceTo(rightBackdrop)
                    && getProfileTime() >= segment.duration
                ) leftBackDropScore++
                else if (robot.distanceTo(leftBackdrop) > robot.distanceTo(rightBackdrop)
                    && getProfileTime() >= segment.duration
                ) rightBackDropScore++
        }
        if (getProfileTime() >= segment.duration) segmentIndex++
    }

    private fun drawTrajectory(gc: GraphicsContext, segment: SequenceSegment) {
        if (segment is TrajectorySegment) {
            gc.lineWidth = 7.0
            gc.stroke = Color.BLUE
            gc.fill = Color.BLUE
            val path = segment.trajectory.path
            val samplePoints = Array(numberSamples) { it * (path.length() / (numberSamples - 1)) }
            val pathPoints = samplePoints.map { point -> path[point].in2px.vec() }
            arrayOf(pathPoints.first(), pathPoints.last()).forEach {
                gc.fillOval(
                    it.x + robotWidth.in2px / 2 - 5,
                    -it.y + robotLength.in2px / 2 - 5,
                    10.0,
                    10.0
                )
            }
            gc.globalAlpha = 0.6
            gc.strokePolyline(
                pathPoints.map { it.x + robotWidth.in2px / 2 }.toDoubleArray(),
                pathPoints.map { -it.y + robotLength.in2px / 2 }.toDoubleArray(),
                numberSamples
            )
            gc.globalAlpha = 1.0
        }
    }

    private fun getTrajectorySequence(): TrajectorySequence {
        return TrajectorySequence(
            listOf(
                backward(2.0),
                strafeLeft(5.5),
                backward(22.0),
                turn((-90.0).toRadians),
                forward(10.0),
                backward(11.0),
                turn((-15.0).toRadians),
                dropPixel(),
                turn(15.0.toRadians),
                strafeRight(35.0),
                turn(180.0.toRadians),
                backward(53.0),
                strafeRight(34.5),
                backward(1.8),
                dropPixelOnBoard()
            )
        )
    }

    private fun forward(distance: Double): TrajectorySegment {
        val segment = newBuilder(prevPose).forward(distance).build()[0] as TrajectorySegment
        segment.trajectoryType = TrajectoryType.FORWARD
        prevPose = segment.endPose
        return segment
    }

    private fun backward(distance: Double): TrajectorySegment {
        val segment = newBuilder(prevPose).back(distance).build()[0] as TrajectorySegment
        segment.trajectoryType = TrajectoryType.BACKWARD
        prevPose = segment.endPose
        return segment
    }

    private fun strafeLeft(distance: Double): TrajectorySegment {
        val segment = newBuilder(prevPose).strafeLeft(distance).build()[0] as TrajectorySegment
        segment.trajectoryType = TrajectoryType.STRAFE_LEFT
        prevPose = segment.endPose
        return segment
    }

    private fun strafeRight(distance: Double): TrajectorySegment {
        val segment = newBuilder(prevPose).strafeRight(distance).build()[0] as TrajectorySegment
        segment.trajectoryType = TrajectoryType.STRAFE_RIGHT
        prevPose = segment.endPose
        return segment
    }

    private fun turn(angle: Double): SequenceSegment {
        val segment = newBuilder(prevPose).turn(angle).build()[0]
        prevPose = segment.endPose
        return segment
    }

    private fun dropPixel() = ActionSegment(Action.DROP_PIXEL, prevPose)
    private fun dropPixelOnBoard() = ActionSegment(Action.DROP_PIXEL_ON_BOARD, prevPose)

    private fun newBuilder(pose: Pose2d) = TrajectorySequenceBuilder(
        pose,
        velocityConstraint,
        accelerationConstraint,
        maxAngVel,
        maxAngAccel
    )

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
val Rectangle2D.centerPointX get() = this.minX + this.width / 2
val Rectangle2D.centerPointY get() = this.minY + this.height / 2

fun main(args: Array<String>) {
    Application.launch(Simulator::class.java, *args)
}
