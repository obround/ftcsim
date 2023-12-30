import javafx.beans.property.SimpleObjectProperty
import javafx.beans.property.SimpleStringProperty
import kotlinx.serialization.Serializable


/**
 * An action to be performed by the robot. Meant to be used for the GUI.
 *
 * @property text: The text to render in the GUI table
 * @property exportableFunction: The 'function' representation of the action for
 *                                when we want to export our trajectory
 */
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


/**
 * A singular movement in the overall trajectory. Meant to be used by the GUI to represent
 * the movement with all of its parameters in the table.
 *
 * @property a: The action performed by the movement
 * @property q: The amount of the action (eg. 42in or 69deg)
 * @property mV: The maximum velocity of the movement ("-" means default)
 * @property mAV: The maximum angular velocity of the movement
 * @property mA: The maximum acceleration of the movement
 */
@Serializable
data class FXTrajectory(
    val a: FXAction,
    val q: String,
    val mV: String = "-",
    val mAV: String = "-",
    val mA: String = "-"
) {
    @Serializable(with = SOPSerializer::class)
    private val action = SimpleObjectProperty(a)

    @Serializable(with = SSPSerializer::class)
    private val quantification = SimpleStringProperty(q)

    @Serializable(with = SSPSerializer::class)
    private val maxVel = SimpleStringProperty(mV)

    @Serializable(with = SSPSerializer::class)
    private val maxAngVel = SimpleStringProperty(mAV)

    @Serializable(with = SSPSerializer::class)
    private val maxAccel = SimpleStringProperty(mA)

    fun newQuantification(q: String) = this.copy(q = q)
    fun newMaxVel(mV: String) = this.copy(mV = mV)
    fun newMaxAngVel(mAV: String) = this.copy(mAV = mAV)
    fun newMaxAccel(mA: String) = this.copy(mA = mA)

    // The get functions may seem unused outside of this class, but they are used internally by JavaFX
    fun getAction(): FXAction = action.get()
    fun getQuantification(): String = quantification.get()
    fun getMaxVel(): String = maxVel.get()
    fun getMaxAngVel(): String = maxAngVel.get()
    fun getMaxAccel(): String = maxAccel.get()
    fun actionProperty() = action

    /**
     * Returns the exportable representation of the action as a string
     */
    fun exportable() = when (val action = getAction()) {
        FXAction.TURN -> "${action.exportableFunction}(${getQuantification()})"
        else -> {
            val mV = getMaxVel().isOrElse("-", "maxVelocity")
            val mAV = getMaxAngVel().isOrElse("-", "maxAngularVelocity")
            val mA = getMaxAccel().isOrElse("-", "maxAcceleration")
            "${action.exportableFunction}(${getQuantification()}, ${mV}, ${mAV}, ${mA}, trackWidth)"
        }
    }

    private fun String.isOrElse(cond: String, newValue: String) = if (this == cond) newValue else this
}
