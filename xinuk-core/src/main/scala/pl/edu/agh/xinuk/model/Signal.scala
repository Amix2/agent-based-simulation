package pl.edu.agh.xinuk.model

import io.jvm.uuid.UUID
import pl.edu.agh.xinuk.config.XinukConfig

import scala.math.sqrt

object AgentMessage {
  def createNew(posX: Double, posY: Double)  = new AgentMessage(1, 8, 50.8, posX, posY)
}

object ObstacleMessage {
  def createNew(xs: Array[Int], ys: Array[Int], points: Int)  = new ObstacleMessage(8, xs, ys, points)
}

trait ObjectMessage extends Ordered[ObjectMessage]
{
  def GetDistanceToLive() : Double;
  def DecreaseDistanceToLive(value: Double) : ObjectMessage;
  override def compare(that: ObjectMessage): Int = Ordering.Double.TotalOrdering.compare(GetDistanceToLive(), that.GetDistanceToLive())
  def IsValid(): Boolean = GetDistanceToLive() > 0;
}

case class AgentMessage(smell: Double, distanceToLive : Double, timeToLive: Double, posX: Double, posY: Double) extends ObjectMessage
{
  override def GetDistanceToLive(): Double = distanceToLive

  override def DecreaseDistanceToLive(value: Double): AgentMessage = AgentMessage(smell, distanceToLive-value, timeToLive, posX, posY);
}

case class ObstacleMessage(distanceToLive : Double, xs: Array[Int], ys: Array[Int], points: Int) extends ObjectMessage
{
  override def GetDistanceToLive(): Double = distanceToLive

  override def DecreaseDistanceToLive(value: Double): ObstacleMessage = ObstacleMessage(distanceToLive-value, xs, ys, points);
}

final case class Signal(value: Double, objectMessages: Map[UUID, ObjectMessage])  {
  def RemoveExpiredMessages(): Signal = {
    Signal(this.value, objectMessages.filter(objMsg => objMsg._2.IsValid()))
  }

  def AddDistanceToAgentMessage(dist: Double, time: Double): Signal = {
    Signal(this.value
      , objectMessages.map(
        { case (id, sig) =>
          (id, sig.DecreaseDistanceToLive(dist))
        }
      ))
  }

  def CombineAgentSignal(signalA : Map[UUID, ObjectMessage], signalB : Map[UUID, ObjectMessage]): Map[UUID, ObjectMessage] =
  {
    val merged = signalA.toSeq ++ signalB.toSeq
    val grouped = merged.groupBy(_._1)
    return grouped.map({ case (key, value) => (key, value.map(T => T._2).max) })
  }
  def +(other: Signal): Signal = Signal(value + other.value, CombineAgentSignal(objectMessages, other.objectMessages))

  def *(factor: Double): Signal = Signal(value * factor, objectMessages)

  private def /(divisor: Double): Signal = Signal(value / divisor, objectMessages)

}

object Signal {
  private final val Zero = Signal(0d, Map.empty[UUID, AgentMessage])

  def zero: Signal = Zero
}

final case class SignalMap(value: Map[Direction, Signal]) extends AnyVal {
  def removeAgentSignals()(implicit config: XinukConfig): SignalMap = {
    this.map(vDS => (vDS._1, Signal(vDS._2.value, Map.empty[UUID, AgentMessage])))
  }

  def apply(direction: Direction): Signal = value(direction)

  def +(other: SignalMap)(implicit config: XinukConfig): SignalMap =
    config.worldType.directions.map(d => (d, this(d) + other(d))).toMap

  def +(added: Signal)(implicit config: XinukConfig): SignalMap =
    config.worldType.directions.map(d => (d, this(d) + added)).toMap

  def applyDistancePropagationUpdate(iteration: Long)(implicit config: XinukConfig): SignalMap = {
    (this * config.signalSuppressionFactor).map(vDS => (vDS._1, vDS._2.AddDistanceToAgentMessage(config.toMeters(config.cellSize) * (if(vDS._1.isDiagonal) sqrt(2) else 1), 0)))
  }

  def applyTimePropagationUpdate(iteration: Long)(implicit config: XinukConfig): SignalMap = {
    (this * config.signalAttenuationFactor).map(vDS => (vDS._1, vDS._2.AddDistanceToAgentMessage(0, config.deltaTime)))
  }
  def applySignalUpdates(oldSignal: SignalMap, iteration: Long)(implicit config: XinukConfig): SignalMap = {
    val sig: SignalMap = (oldSignal.removeAgentSignals() + this.applyDistancePropagationUpdate(iteration)).applyTimePropagationUpdate(iteration)
    //val sigNew = sig.map(vDS => (vDS._1, vDS._2.AddDistanceToAgentMessage(1, 1)))
    sig.map(vDS => (vDS._1, vDS._2.RemoveExpiredMessages()))

  }

  //  def -(other: SignalMap)(implicit config: XinukConfig): SignalMap =
//    config.worldType.directions.map(d => (d, this(d) - other(d))).toMap

//  def -(deducted: Signal)(implicit config: XinukConfig): SignalMap =
//    config.worldType.directions.map(d => (d, this(d) - deducted)).toMap

  def *(factor: Double)(implicit config: XinukConfig): SignalMap =
    config.worldType.directions.map(d => (d, this(d) * factor)).toMap

//  def /(divisor: Double)(implicit config: XinukConfig): SignalMap =
//    config.worldType.directions.map(d => (d, this(d) / divisor)).toMap
}

object SignalMap {
  implicit def map2SignalMap(map: Map[Direction, Signal]): SignalMap =
    SignalMap(map)

  implicit def signalMap2Map(signalMap: SignalMap): Map[Direction, Signal] =
    signalMap.value

  def empty(implicit config: XinukConfig): SignalMap =
    uniform(Signal.zero)

  def uniform(initialSignal: Signal)(implicit config: XinukConfig): SignalMap =
    config.worldType.directions.map(d => (d, initialSignal)).toMap
}
