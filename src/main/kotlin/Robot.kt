import com.acmerobotics.roadrunner.geometry.Pose2d
import javafx.geometry.Rectangle2D
import javafx.scene.canvas.GraphicsContext
import javafx.scene.image.Image
import javafx.scene.transform.Rotate
import kotlin.math.pow
import kotlin.math.sqrt


/**
 * Helper class meant to represent the Robot on the canvas. Takes care of rendering it and
 * detecting collisions with other objects on the field.
 *
 * @property image: The robot's image
 * @property fieldDimPixels: The size of the field image in pixels
 * @param startPose: The starting position of the robot (x:pixels, y:pixels, heading:degrees)
 */
class Robot(private val image: Image, private val fieldDimPixels: Double, startPose: Pose2d) {
    var x = startPose.x
        set(value) {
            field = value.coerceIn(0.0, fieldDimPixels) // Ensures that the robot stays within the field
        }
    var y = -startPose.y
        set(value) {
            field = value.coerceIn(0.0, fieldDimPixels)  // Ensures that the robot stays within the field
        }
    var rotation = startPose.heading  // degrees
    val asRectangle2D get() = Rectangle2D(x, y, image.width, image.height)
    private val centerPointX get() = x + image.width / 2
    private val centerPointY get() = y + image.height / 2

    /**
     * Renders the robot on the canvas taking care of position and rotation.
     */
    fun render(gc: GraphicsContext) {
        gc.save()
        // Rotation about pivot point
        val r = Rotate(-rotation, centerPointX, centerPointY)
        gc.setTransform(r.mxx, r.myx, r.mxy, r.myy, r.tx, r.ty)
        gc.drawImage(image, x, y)
        gc.restore()
    }

    fun moveTo(pose: Pose2d) {
        x = pose.x
        y = -pose.y
        rotation = pose.heading
    }

    /**
     * Identifies if the robot is colliding with another object
     */
    fun collision(other: Rectangle2D) = other.intersects(asRectangle2D)

    /**
     * Simple pythagorean formula to determine the distance ot another object
     */
    fun distanceTo(other: Rectangle2D) =
        sqrt((centerPointX - other.centerPointX).pow(2) + (centerPointY - other.centerPointY).pow(2))
}
