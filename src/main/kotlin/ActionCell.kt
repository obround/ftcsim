import javafx.scene.control.ComboBox
import javafx.scene.control.TableCell


class ActionCell(private var combo: ComboBox<String>) : TableCell<FXTrajectory, String>() {
    override fun updateItem(reason: String?, empty: Boolean) {
        super.updateItem(reason, empty)
        graphic = if (empty) {
            null
        } else {
            combo.value = reason
            combo
        }
    }
}

