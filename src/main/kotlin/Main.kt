import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint
import javafx.animation.KeyFrame
import javafx.animation.Timeline
import javafx.application.Application
import javafx.collections.FXCollections
import javafx.geometry.Insets
import javafx.geometry.Pos
import javafx.geometry.Rectangle2D
import javafx.geometry.VPos
import javafx.scene.Scene
import javafx.scene.canvas.Canvas
import javafx.scene.canvas.GraphicsContext
import javafx.scene.control.*
import javafx.scene.control.cell.PropertyValueFactory
import javafx.scene.control.cell.TextFieldTableCell
import javafx.scene.image.Image
import javafx.scene.image.ImageView
import javafx.scene.input.ClipboardContent
import javafx.scene.input.DataFormat
import javafx.scene.input.TransferMode
import javafx.scene.layout.HBox
import javafx.scene.layout.Priority
import javafx.scene.layout.VBox
import javafx.scene.paint.Color
import javafx.scene.text.Font
import javafx.scene.text.TextAlignment
import javafx.stage.FileChooser
import javafx.stage.Stage
import javafx.util.Duration
import kotlinx.serialization.encodeToString
import kotlinx.serialization.json.Json
import trajectorysequence.TrajectorySequence
import trajectorysequence.TrajectorySequenceBuilder
import trajectorysequence.TrajectoryType
import trajectorysequence.sequencesegment.SequenceSegment
import trajectorysequence.sequencesegment.TrajectorySegment
import trajectorysequence.sequencesegment.TurnSegment
import trajectorysequence.sequencesegment.WaitSegment
import java.io.File
import kotlin.math.cos
import kotlin.math.min
import kotlin.math.sin


class Simulator : Application() {
    // CONSTANTS
    private val fps = 60  // Frames per second
    private val numberSamples = 50  // Number of samples when drawing the trajectory

    // TODO: Make this changeable
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
        fieldDimPixels,
        startPose.in2px
    )

    // OTHER
    private var simulate = false
    private var leftBackDropScore = 0
    private var rightBackDropScore = 0
    private var pixelPositions = mutableListOf<Pair<Double, Double>>()
    private var prevPose = startPose
    private val velocityConstraint = createVelocityConstraint(maxVel, maxAngVel)
    private val accelerationConstraint = createAccelerationConstraint(maxAccel)
    private var startTime = Double.NaN
    private val trajectoryTable = TableView<FXTrajectory>()

    private var sequence = getTrajectorySequence()
    private var segmentIndex = 0
    private var trajectoryDurations = sequence.sequenceList?.map { it.duration } ?: emptyList()
    private var totalDuration = trajectoryDurations.sum()

    // CONVERSIONS
    private val Double.in2px get() = this * fieldDimPixels / fieldDimReal
    private val Pose2d.in2px get() = Pose2d(this.x.in2px, this.y.in2px, heading.toDegrees)

    private val windowWidth = field.width + 600
    private val windowHeight = field.height + 50

    // UI
    private val serializedMimeType = DataFormat("application/x-java-serialized-object")
    private var trajectoriesModified = false
    private val selections = mutableListOf<FXTrajectory>()
    private val root = HBox()
    private val canvas = Canvas(field.width, field.height)
    private val builder = VBox()
    private lateinit var timeline: Timeline
    private val timer = ProgressBar(0.0)
    private val timeCount = Label("- / -")
    private val collisionError = Alert(
        Alert.AlertType.ERROR,
        "ur skill issue got the opps ded crying",
        ButtonType.OK
    )
    private val expectedDoubleError = Alert(
        Alert.AlertType.ERROR,
        "why tf would u not put a positive decimal bruh",
        ButtonType.OK
    )
    private val actionOptions =
        FXCollections.observableList(
            listOf(
                FXAction.FORWARD,
                FXAction.BACKWARD,
                FXAction.STRAFE_LEFT,
                FXAction.STRAFE_RIGHT,
                FXAction.TURN,
                FXAction.DROP_PIXEL,
                FXAction.DROP_PIXEL_ON_BOARD
            )
        )

    override fun start(stage: Stage) {
        root.padding = Insets(25.0)
        builder.padding = Insets(0.0, 0.0, 0.0, 25.0)
        builder.spacing = 25.0
        HBox.setHgrow(builder, Priority.ALWAYS)

        initLogo()
        initTrajectoryTable()
        // Requires the stage to create the file dialogs for save/open
        initButtons(stage)
        initTimer()
        initErrors()

        stage.title = "FTCSIM"
        stage.isResizable = false
        canvas.graphicsContext2D.isImageSmoothing = false

        timeline = Timeline(KeyFrame(
            Duration.millis(1000.0 / fps),
            { simulationLoop(canvas.graphicsContext2D) }
        ))
        timeline.cycleCount = Timeline.INDEFINITE

        root.children.addAll(canvas, builder)
        stage.scene = Scene(root, windowWidth, windowHeight)
        stage.show()
        timeline.play()
    }

    private fun initErrors() {
        collisionError.title = null
        collisionError.headerText = "Invalid collision with fixed object"
        expectedDoubleError.title = null
        expectedDoubleError.headerText = "Expected a decimal value for the specified field"
    }

    private fun initTimer() {
        val timerUnit = HBox()

        HBox.setHgrow(timer, Priority.ALWAYS)
        timer.maxWidth = Double.MAX_VALUE
        timerUnit.spacing = 5.0

        timerUnit.children.addAll(timer, timeCount)
        builder.children.add(timerUnit)
    }

    private fun initButtons(stage: Stage) {
        val buttonBar = HBox()
        val add = Button("Add")
        val remove = Button("Remove")
        val run = Button("Run")
        val stop = Button("Stop")
        val save = Button("Save")
        val open = Button("Open")
        val export = Button("Export")

        buttonBar.spacing = 4.0

        setupImageButton(add, "/add.png")
        setupImageButton(remove, "/remove.png")
        setupImageButton(run, "/run.png")
        setupImageButton(stop, "/stop.png")
        setupImageButton(save, "/save.png")
        setupImageButton(open, "/open.png")
        setupImageButton(export, "/export.png")

        add.setOnAction {
            trajectoryTable.items.add(FXTrajectory(FXAction.FORWARD, "10"))
            trajectoriesModified = true
        }
        remove.setOnAction {
            // We sort the indices in ascending order, and THEN remove the values from the table to avoid shifting the
            // other indices out of their position
            trajectoryTable.selectionModel.selectedIndices.sorted().reversed()
                .forEach { trajectoryTable.items.removeAt(it) }
            trajectoriesModified = true
        }
        run.setOnAction {
            if (trajectoriesModified) stopSimulation()
            timeline.play()
            simulate = true
        }
        stop.setOnAction { stopSimulation() }
        save.setOnAction { saveTrajectory(stage) }
        open.setOnAction { openTrajectory(stage) }
        export.setOnAction { exportTrajectory() }

        buttonBar.children.addAll(add, remove, run, stop, save, open, export)
        builder.children.add(buttonBar)
    }

    private fun initTextColumn(
        col: TableColumn<FXTrajectory, String>,
        propertyName: String,
        sideEffect: (FXTrajectory, String) -> FXTrajectory
    ) {
        col.style = "-fx-alignment: CENTER-LEFT;"
        col.cellValueFactory = PropertyValueFactory(propertyName)
        col.cellFactory = TextFieldTableCell.forTableColumn()
        col.setOnEditCommit { t ->
            val value = t.newValue.toDoubleOrNull()
            val prevValue = trajectoryTable.items[t.tablePosition.row]
            // Note that turns are currently allowed to have negative values
            if ((value != null && (value > 0 || prevValue.getAction() == FXAction.TURN)) || t.newValue == "-") {
                val position = t.tablePosition.row
                val prevTrajectory = trajectoryTable.items[position].copy()
                // To whoever is maintaining this code, I DARE YOU to simplify these two lines; Even if you're smart,
                // and replace it with trajectoryTable.refresh(), it's not going to refresh the table internally
                trajectoryTable.items.removeAt(position)
                trajectoryTable.items.add(position, sideEffect(prevTrajectory, t.newValue))
                trajectoriesModified = true
            } else expectedDoubleError.showAndWait()
        }
        col.isSortable = false
    }

    private fun initTrajectoryTable() {
        val actionColumn = TableColumn<FXTrajectory, FXAction>("Action")
        val quantificationColumn = TableColumn<FXTrajectory, String>("in/deg")
        val maxVelColumn = TableColumn<FXTrajectory, String>("vₘₐₓ")
        val maxAngVelColumn = TableColumn<FXTrajectory, String>("ωₘₐₓ")
        val maxAccelColumn = TableColumn<FXTrajectory, String>("aₘₐₓ")

        initTextColumn(quantificationColumn, "Quantification", FXTrajectory::newQuantification)
        initTextColumn(maxVelColumn, "MaxVel", FXTrajectory::newMaxVel)
        initTextColumn(maxAngVelColumn, "MaxAngVel", FXTrajectory::newMaxAngVel)
        initTextColumn(maxAccelColumn, "MaxAccel", FXTrajectory::newMaxAccel)

        actionColumn.minWidth = 70.0
        actionColumn.setCellValueFactory { cellData -> cellData.value.actionProperty() }
        actionColumn.setCellFactory {
            val combo: ComboBox<FXAction> = ComboBox(actionOptions)
            val cell = ActionCell(combo)
            combo.setOnAction {
                val prevTrajectory = trajectoryTable.items[cell.index].copy()
                trajectoryTable.items.removeAt(cell.index)
                trajectoryTable.items.add(cell.index, prevTrajectory.newAction(combo.value))
                trajectoriesModified = true
            }
            cell
        }
        actionColumn.isSortable = false

        // To make the table draggable
        trajectoryTable.setRowFactory {
            val row: TableRow<FXTrajectory> = TableRow()
            row.setOnDragDetected { event ->
                if (!row.isEmpty) {
                    val index = row.index
                    selections.clear()
                    val items = trajectoryTable.selectionModel.selectedItems
                    items.forEach { selections.add(it) }
                    val db = row.startDragAndDrop(TransferMode.MOVE)
                    db.dragView = row.snapshot(null, null)
                    val cc = ClipboardContent()
                    cc[serializedMimeType] = index
                    db.setContent(cc)
                    event.consume()
                }
            }
            row.setOnDragOver { event ->
                val db = event.dragboard
                if (db.hasContent(serializedMimeType) && row.index != db.getContent(serializedMimeType) as Int) {
                    event.acceptTransferModes(*TransferMode.COPY_OR_MOVE)
                    event.consume()
                }
            }
            row.setOnDragDropped { event ->
                val db = event.dragboard
                if (db.hasContent(serializedMimeType)) {
                    var dI: FXTrajectory? = null
                    var dropIndex = if (row.isEmpty) {
                        trajectoryTable.items.size
                    } else {
                        dI = trajectoryTable.items[row.index]
                        row.index
                    }
                    var delta = 0
                    if (dI != null) while (selections.contains(dI)) {
                        delta = 1
                        --dropIndex
                        if (dropIndex < 0) {
                            dI = null
                            dropIndex = 0
                            break
                        }
                        dI = trajectoryTable.items[dropIndex]
                    }
                    selections.forEach { trajectoryTable.items.remove(it) }
                    if (dI != null) dropIndex = trajectoryTable.items.indexOf(dI) + delta
                    else if (dropIndex != 0) dropIndex = trajectoryTable.items.size
                    trajectoryTable.selectionModel.clearSelection()
                    selections.forEach {
                        trajectoryTable.items.add(dropIndex, it)
                        trajectoryTable.selectionModel.select(dropIndex)
                        dropIndex++
                    }
                    event.isDropCompleted = true
                    selections.clear()
                    event.consume()
                }
            }
            row
        }

        trajectoryTable.items.add(FXTrajectory(FXAction.FORWARD, "10"))
        trajectoryTable.columns.addAll(
            actionColumn,
            quantificationColumn,
            maxVelColumn,
            maxAngVelColumn,
            maxAccelColumn
        )
        trajectoryTable.selectionModel.selectionMode = SelectionMode.MULTIPLE
        trajectoryTable.columnResizePolicy = TableView.CONSTRAINED_RESIZE_POLICY
        trajectoryTable.isEditable = true
        trajectoryTable.placeholder = Label("No trajectories added")

        builder.children.add(trajectoryTable)
    }

    private fun initLogo() {
        val topLabel = HBox()
        val ftcLogo = ImageView("/ftc_logo.png")
        val ftcsimText = Label("FTCSIM")

        ftcsimText.font = Font("Arial Bold Italic", 72.0)
        ftcLogo.fitWidth = 150.0
        ftcLogo.isPreserveRatio = true
        topLabel.children.addAll(ftcLogo, ftcsimText)
        topLabel.alignment = Pos.CENTER

        builder.children.add(topLabel)
    }

    private fun saveTrajectory(stage: Stage) {
        val serialized = Json.encodeToString(trajectoryTable.items.toList())
        val fileChooser = FileChooser()
        fileChooser.title = "Save"
        fileChooser.initialFileName = "trajectory.ftcsim"
        val selectedFile = fileChooser.showSaveDialog(stage)
        if (selectedFile != null)
            File(selectedFile.toPath().toString()).printWriter().use { out -> out.println(serialized) }
    }

    private fun openTrajectory(stage: Stage) {
        val fileChooser = FileChooser()
        fileChooser.title = "Open"
        val selectedFile = fileChooser.showOpenDialog(stage)
        if (selectedFile != null) {
            val text = File(selectedFile.toPath().toString()).readText()
            val deserialized = Json.decodeFromString<List<FXTrajectory>>(text)
            // TODO: Add prompt asking "are you sure you want to delete the current trajectories"
            trajectoryTable.items.clear()
            trajectoryTable.items.addAll(deserialized)
        }
        stopSimulation()
    }

    private fun exportTrajectory() {
        val body = trajectoryTable.items.joinToString(separator = "\n    ") { it.exportable() + ";" }
        val export = "private void movement() {\n    $body\n}"
        val popup = Stage()

        val copyableField = TextArea()
        copyableField.setPrefSize(600.0, 400.0)
        copyableField.isEditable = false
        copyableField.text = export

        popup.scene = Scene(copyableField)
        popup.show()
    }

    private fun simulationLoop(gc: GraphicsContext) {
        if (trajectoriesModified) stopSimulation()
        gc.drawImage(field, 0.0, 0.0)
        if (simulate) {
            when (val segment = sequence.get(segmentIndex)) {
                is TrajectorySegment -> updateRobotMove(segment)
                is TurnSegment -> updateRobotTurn(segment)
                is ActionSegment -> updateRobotAction(segment)
                is WaitSegment -> TODO()
            }
        }
        // Render everything
        robot.render(gc)
        // Draw the robot's trajectory
        sequence.sequenceList?.forEach { drawTrajectory(gc, it) }
        // Draw the backboard score
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
        // Draw the pixels placed on the field
        pixelPositions.forEach { (x, y) -> gc.drawImage(pixel, x - pixel.width / 2, y - pixel.height / 2) }
        handleCollisions()
        // Stop the animation once we finish the final segment
        if (segmentIndex == sequence.size()) {
            timeline.stop()
            resetValues()
        }
    }

    private fun stopSimulation() {
        simulate = false
        timer.progress = 0.0
        resetValues()
        robot.moveTo(startPose.in2px)
        timeline.play()
    }

    private fun resetValues() {
        leftBackDropScore = 0
        rightBackDropScore = 0
        pixelPositions = mutableListOf()
        prevPose = startPose
        startTime = Double.NaN
        sequence = getTrajectorySequence()
        trajectoryDurations = sequence.sequenceList?.map { it.duration } ?: emptyList()
        totalDuration = trajectoryDurations.sum()
        timeCount.text = "- / %.1fs".format(totalDuration)
        segmentIndex = 0
        trajectoriesModified = false
    }

    private fun handleCollisions() {
        fixedObjects.forEach { obj ->
            if (robot.collision(obj)) {
                timeline.stop()
                collisionError.show()
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
        return TrajectorySequence(trajectoryTable.items.map { (action, q, mV, mAV, mA) ->
            val amount = q.toDouble()
            val maxVel = if (mV == "-") maxVel else mV.toDouble()
            val maxAngVel = if (mAV == "-") maxAngVel else mAV.toDouble()
            val maxAccel = if (mA == "-") maxAccel else mA.toDouble()
            when (action) {
                FXAction.FORWARD -> forward(amount, maxVel, maxAngVel, maxAccel)
                FXAction.BACKWARD -> backward(amount, maxVel, maxAngVel, maxAccel)
                FXAction.STRAFE_LEFT -> strafeLeft(amount, maxVel, maxAngVel, maxAccel)
                FXAction.STRAFE_RIGHT -> strafeRight(amount, maxVel, maxAngVel, maxAccel)
                FXAction.TURN -> turn(amount.toRadians)
                FXAction.DROP_PIXEL -> dropPixel()
                FXAction.DROP_PIXEL_ON_BOARD -> dropPixelOnBoard()
            }
        })
    }

    private fun forward(distance: Double, maxVel: Double, maxAngVel: Double, maxAccel: Double): TrajectorySegment {
        val velConst = createVelocityConstraint(maxVel, maxAngVel)
        val accelConst = createAccelerationConstraint(maxAccel)
        val segment = newBuilder(prevPose).forward(distance, velConst, accelConst).build()[0] as TrajectorySegment
        segment.trajectoryType = TrajectoryType.FORWARD
        prevPose = segment.endPose
        return segment
    }

    private fun backward(distance: Double, maxVel: Double, maxAngVel: Double, maxAccel: Double): TrajectorySegment {
        val velConst = createVelocityConstraint(maxVel, maxAngVel)
        val accelConst = createAccelerationConstraint(maxAccel)
        val segment = newBuilder(prevPose).back(distance, velConst, accelConst).build()[0] as TrajectorySegment
        segment.trajectoryType = TrajectoryType.BACKWARD
        prevPose = segment.endPose
        return segment
    }

    private fun strafeLeft(distance: Double, maxVel: Double, maxAngVel: Double, maxAccel: Double): TrajectorySegment {
        val velConst = createVelocityConstraint(maxVel, maxAngVel)
        val accelConst = createAccelerationConstraint(maxAccel)
        val segment = newBuilder(prevPose).strafeLeft(distance, velConst, accelConst).build()[0] as TrajectorySegment
        segment.trajectoryType = TrajectoryType.STRAFE_LEFT
        prevPose = segment.endPose
        return segment
    }

    private fun strafeRight(distance: Double, maxVel: Double, maxAngVel: Double, maxAccel: Double): TrajectorySegment {
        val velConst = createVelocityConstraint(maxVel, maxAngVel)
        val accelConst = createAccelerationConstraint(maxAccel)
        val segment = newBuilder(prevPose).strafeRight(distance, velConst, accelConst).build()[0] as TrajectorySegment
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
        val deltaTime = time - startTime
        timer.progress = deltaTime / totalDuration
        timeCount.text = "%.1fs / %.1fs".format(min(deltaTime, totalDuration), totalDuration)
        return deltaTime - prevTrajectoryDuration
    }

    private fun createVelocityConstraint(maxVel: Double, maxAngVel: Double) =
        MinVelocityConstraint(
            listOf(
                AngularVelocityConstraint(maxAngVel),
                MecanumVelocityConstraint(
                    maxVel,
                    trackWidth
                )
            )
        )

    private fun createAccelerationConstraint(maxAccel: Double) = ProfileAccelerationConstraint(maxAccel)

    private fun setupImageButton(button: Button, imagePath: String) {
        val image = ImageView(imagePath)
        button.setPrefSize(30.0, 30.0)
        image.isPreserveRatio = true
        image.fitWidth = button.prefWidth
        button.graphic = image
        button.contentDisplay = ContentDisplay.GRAPHIC_ONLY
    }
}

val Double.toRadians get() = Math.toRadians(this)
val Double.toDegrees get() = Math.toDegrees(this)
val Rectangle2D.centerPointX get() = this.minX + this.width / 2
val Rectangle2D.centerPointY get() = this.minY + this.height / 2


fun main(args: Array<String>) {
    Application.launch(Simulator::class.java, *args)
}
