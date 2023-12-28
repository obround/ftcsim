import javafx.scene.control.ComboBox
import javafx.scene.control.TableCell


class ActionCell(private var combo: ComboBox<String>) : TableCell<FXTrajectory, String>() {
    override fun updateItem(act: String?, empty: Boolean) {
        super.updateItem(act, empty)
        graphic = if (empty) {
            null
        } else {
            combo.value = act
            combo
        }
    }
}
