import javafx.beans.property.SimpleIntegerProperty
import javafx.beans.property.SimpleStringProperty

class FXTrajectory(a: String, q: String) {
    private val action = SimpleStringProperty(a)
    private val quantification = SimpleStringProperty(q)

    fun getAction(): String = action.get()
    fun getQuantification(): String = quantification.get()
    fun setAction(new: String) { action.set(new) }
    fun setQuantification(new: String) { quantification.set(new) }
}
