import javafx.beans.property.SimpleIntegerProperty
import javafx.beans.property.SimpleStringProperty

class FXTrajectory(a: String, q: Int) {
    private val action = SimpleStringProperty(a)
    private val quantification = SimpleIntegerProperty(q)

    fun getAction(): String = action.get()
    fun getQuantification(): Int = quantification.get()
    fun setAction(new: String) { action.set(new) }
    fun setQuantification(new: Int) { quantification.set(new) }
}
