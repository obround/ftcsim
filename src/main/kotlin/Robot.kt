import javafx.geometry.Rectangle2D
import javafx.scene.canvas.GraphicsContext
import javafx.scene.image.Image
import javafx.scene.transform.Rotate
import kotlin.math.pow
import kotlin.math.sqrt


class Robot(private val image: Image, private val fieldDimPixels: Double) {
    var x = 0.0
        set(value) {
            field = value.coerceIn(0.0, fieldDimPixels)
        }
    var y = 0.0
        set(value) {
            field = value.coerceIn(0.0, fieldDimPixels)
        }
    var rotation = 0.0  // degrees
    val asRectangle2D get() = Rectangle2D(x, y, image.width, image.height)
    private val centerPointX get() = x + image.width / 2
    private val centerPointY get() = y + image.height / 2

    fun render(gc: GraphicsContext) {
        gc.save()
        // Rotation about pivot point
        val r = Rotate(-rotation, centerPointX, centerPointY)
        gc.setTransform(r.mxx, r.myx, r.mxy, r.myy, r.tx, r.ty)
        gc.drawImage(image, x, y)
        gc.restore()
    }

    fun collision(other: Rectangle2D) = other.intersects(asRectangle2D)

    fun distanceTo(other: Rectangle2D) =
        sqrt((centerPointX - other.centerPointX).pow(2) + (centerPointY - other.centerPointY).pow(2))
}
