import javafx.beans.property.SimpleObjectProperty
import javafx.beans.property.SimpleStringProperty
import kotlinx.serialization.KSerializer
import kotlinx.serialization.descriptors.PrimitiveKind
import kotlinx.serialization.descriptors.PrimitiveSerialDescriptor
import kotlinx.serialization.encoding.Decoder
import kotlinx.serialization.encoding.Encoder


/**
 * Serializer for the SimpleObjectProperty class of FXAction. Simply serializes/deserializes the
 * string form of the FXAction.
 */
object SOPSerializer : KSerializer<SimpleObjectProperty<FXAction>> {
    override val descriptor = PrimitiveSerialDescriptor("SimpleObjectProperty", PrimitiveKind.STRING)

    override fun serialize(encoder: Encoder, value: SimpleObjectProperty<FXAction>) {
        encoder.encodeString(value.get().toSerialize())
    }

    override fun deserialize(decoder: Decoder): SimpleObjectProperty<FXAction> {
        return SimpleObjectProperty(FXAction.valueOf(decoder.decodeString()))
    }
}

/**
 * Serializer for the SimpleStringProperty. Simply serializes/deserializes its string parameter.
 */
object SSPSerializer : KSerializer<SimpleStringProperty> {
    override val descriptor = PrimitiveSerialDescriptor("SimpleObjectProperty", PrimitiveKind.STRING)

    override fun serialize(encoder: Encoder, value: SimpleStringProperty) {
        encoder.encodeString(value.get())
    }

    override fun deserialize(decoder: Decoder): SimpleStringProperty {
        return SimpleStringProperty(decoder.decodeString())
    }
}
