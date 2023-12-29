import javafx.beans.property.SimpleObjectProperty
import javafx.beans.property.SimpleStringProperty
import kotlinx.serialization.Serializable


enum class FXAction(private val text: String, val exportableFunction: String) {
    FORWARD("Forward", "forward"),
    BACKWARD("Backward", "backward"),
    STRAFE_LEFT("Strafe Left", "strafeLeft"),
    STRAFE_RIGHT("Strafe Right", "strafeRight"),
    TURN("Turn", "turn"),
    DROP_PIXEL("Drop Pixel (Field)", "dropPixel"),
    DROP_PIXEL_ON_BOARD("Drop Pixel (Board)", "dropPixelOnBoard");

    override fun toString() = text
    fun toSerialize() = super.toString()
}


@Serializable
data class FXTrajectory(val a: FXAction, val q: String) {
    @Serializable(with = SOPSerializer::class)
    private val action = SimpleObjectProperty(a)

    @Serializable(with = SSPSerializer::class)
    private val quantification = SimpleStringProperty(q)

    // The get functions may seem unused, but they are used internally by JavaFX
    fun getAction(): FXAction = action.get()
    fun getQuantification(): String = quantification.get()
    fun actionProperty() = action

    fun exportable() = when (val action = getAction()) {
        FXAction.TURN -> "${action.exportableFunction}(${getQuantification()})"
        else -> "${action.exportableFunction}(${getQuantification()}, strafeVelocity, maxAngularVelocity, strafeAcceleration, trackWidth)"
    }
}
