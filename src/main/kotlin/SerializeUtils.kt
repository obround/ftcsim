import com.acmerobotics.roadrunner.geometry.Pose2d
import javafx.beans.property.SimpleObjectProperty
import javafx.beans.property.SimpleStringProperty
import kotlinx.serialization.KSerializer
import kotlinx.serialization.Serializable
import kotlinx.serialization.builtins.DoubleArraySerializer
import kotlinx.serialization.descriptors.PrimitiveKind
import kotlinx.serialization.descriptors.PrimitiveSerialDescriptor
import kotlinx.serialization.encoding.*


/**
 * Tuple containing the entire state of the simulator when the trajectory is saved.
 */
@Serializable
data class SerializeQuintuple(
    val trajectory: List<FXTrajectory>,
    @Serializable(with = Pose2dSerializer::class) val startPose: Pose2d,
    val specifiedMaxVel: Double,
    val specifiedMaxAngVel: Double,
    val specifiedMaxAccel: Double
)


/**
 * Serializer for the `SimpleObjectProperty` class of `FXAction`. Simply serializes/deserializes the
 * string form of the `FXAction`.
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
 * Serializer for the `SimpleStringProperty`. Simply serializes/deserializes its string parameter.
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


/**
 * Serializer for `Pose2d`. Since `Pose2d` is a data class, it should be possible to simply annotate it with
 * `@Serializable`, however it is contained in the roadrunner library which we shouldn't modify, so instead
 * we explicitly create a serializer for it.
 */
object Pose2dSerializer : KSerializer<Pose2d> {
    private val delegateSerializer = DoubleArraySerializer()
    override val descriptor = PrimitiveSerialDescriptor("Pose2d", PrimitiveKind.STRING)

    override fun serialize(encoder: Encoder, value: Pose2d) {
        encoder.encodeSerializableValue(delegateSerializer, doubleArrayOf(value.x, value.y, value.heading))
    }

    override fun deserialize(decoder: Decoder): Pose2d {
        val array = decoder.decodeSerializableValue(delegateSerializer)
        return Pose2d(array[0], array[1], array[2])
    }
}
