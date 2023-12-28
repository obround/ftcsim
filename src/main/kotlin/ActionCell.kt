import javafx.scene.control.ComboBox
import javafx.scene.control.TableCell


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
