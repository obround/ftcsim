import javafx.beans.property.SimpleObjectProperty
import javafx.beans.property.SimpleStringProperty
import kotlinx.serialization.Serializable


@Serializable
enum class FXAction(private val text: String) {
    FORWARD("Forward"),
    BACKWARD("Backward"),
    STRAFE_LEFT("Strafe Left"),
    STRAFE_RIGHT("Strafe Right"),
    TURN("Turn"),
    DROP_PIXEL("Drop Pixel (Field)"),
    DROP_PIXEL_ON_BOARD("Drop Pixel (Board)");

    override fun toString() = text
    fun toSerialize() = super.toString()
}


@Serializable
data class FXTrajectory(val a: FXAction, val q: String) {
    @Serializable(with = SOPSerializer::class)
    private val action = SimpleObjectProperty(a)
    @Serializable(with = SSPSerializer::class)
    private val quantification = SimpleStringProperty(q)

    // The get functions seem unused, but they are used internally by JavaFX
    fun getAction(): FXAction = action.get()
    fun getQuantification(): String = quantification.get()
    fun actionProperty() = action
}
