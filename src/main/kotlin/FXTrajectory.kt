import javafx.beans.property.SimpleObjectProperty
import javafx.beans.property.SimpleStringProperty


enum class FXAction(val text: String) {
    FORWARD("Forward"),
    BACKWARD("Backward"),
    STRAFE_LEFT("Strafe Left"),
    STRAFE_RIGHT("Strafe Right"),
    TURN("Turn"),
    DROP_PIXEL("Drop Pixel (Field)"),
    DROP_PIXEL_ON_BOARD("Drop Pixel (Board)");

    override fun toString() = text
}


data class FXTrajectory(val a: FXAction, val q: String) {
    // TODO: Switch to enums for the action (with ObjectProperty)
    private val action = SimpleObjectProperty(a)
    private val quantification = SimpleStringProperty(q)

    // The get functions seem unused, but they are used internally by JavaFX
    fun getAction(): FXAction = action.get()
    fun getQuantification(): String = quantification.get()

    // These functions create a new FXTrajectory because while they are stored in the table, they are apparently
    // immutable, and this was causing some nasty bugs. So patchy solution, this is a TODO
    fun newAction(new: FXAction) = FXTrajectory(new, getQuantification())
    fun newQuantification(new: String) = FXTrajectory(getAction(), new)
    fun actionAsString() = action
}
