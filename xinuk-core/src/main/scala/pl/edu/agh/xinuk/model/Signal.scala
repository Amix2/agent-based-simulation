package pl.edu.agh.xinuk.model

import io.jvm.uuid.UUID
import pl.edu.agh.xinuk.config.XinukConfig

import scala.math.sqrt

object AgentSignal {
  def createNew(posX: Double, posY: Double)  = new AgentSignal(1, 6, 50.8, posX, posY)
}
case class AgentSignal(smell: Double, distanceToLive : Double,  timeToLive: Double, posX: Double, posY: Double) extends Ordered[AgentSignal]
{
  def *(factor: Double): AgentSignal = AgentSignal(smell * factor, distanceToLive, timeToLive, posX, posY);
  def removeDistanceAndTime(dist : Double, time : Double): AgentSignal = AgentSignal(smell, distanceToLive-dist, timeToLive-time, posX, posY);

  override def compare(that: AgentSignal): Int = Ordering.Double.TotalOrdering.compare(smell, that.smell)
}


final case class Signal(value: Double, agentSignals: Map[UUID, AgentSignal]) extends Ordered[Signal] {
  def RemoveExpiredMessages(): Signal = {
    Signal(this.value, agentSignals.filter(agentSig => agentSig._2.timeToLive > 0 && agentSig._2.distanceToLive > 0))
  }

  def AddDistanceToAgentMessage(dist: Double, time: Double): Signal = {
    Signal(this.value
      , agentSignals.map(
        { case (id, sig) =>
          (id, sig.removeDistanceAndTime(dist, time))
        }
      ))
  }

  def CombineAgentSignal(signalA : Map[UUID, AgentSignal], signalB : Map[UUID, AgentSignal]): Map[UUID, AgentSignal] =
  {
    val merged = signalA.toSeq ++ signalB.toSeq
    val grouped = merged.groupBy(_._1)
    var groupedCleared = grouped.map({ case (key, value) => (key, value.map(T => T._2).max) })
    groupedCleared
  }
  def +(other: Signal): Signal = Signal(value + other.value, CombineAgentSignal(agentSignals, other.agentSignals))

  //def -(other: Signal): Signal = Signal(value - other.value, agentSignals)

  def MulAgentSignal(factor : Double): ((UUID, AgentSignal)) => (UUID, AgentSignal) = {
    case (key, value) =>
      key -> value * factor
  }
  def *(factor: Double): Signal = Signal(value * factor, agentSignals.map(MulAgentSignal(factor)))

  private def /(divisor: Double): Signal = Signal(value / divisor, agentSignals.map(MulAgentSignal(1/divisor)))

   override def compare(that: Signal): Int = Ordering.Double.TotalOrdering.compare(value, that.value)
}

object Signal {
  private final val Zero = Signal(0d, Map.empty[UUID, AgentSignal])

  def zero: Signal = Zero
}

final case class SignalMap(value: Map[Direction, Signal]) extends AnyVal {
  def removeAgentSignals()(implicit config: XinukConfig): SignalMap = {
    this.map(vDS => (vDS._1, Signal(vDS._2.value, Map.empty[UUID, AgentSignal])))
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
