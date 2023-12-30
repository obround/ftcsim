import javafx.scene.control.ComboBox
import javafx.scene.control.TableCell


/**
 * A cell with a ComboBox in it. Meant for the action column.
 */
class ActionCell(private var combo: ComboBox<FXAction>) : TableCell<FXTrajectory, FXAction>() {
    override fun updateItem(act: FXAction?, empty: Boolean) {
        super.updateItem(act, empty)
        graphic = if (empty) {
            null
        } else {
            combo.value = act
            combo
        }
    }
}
