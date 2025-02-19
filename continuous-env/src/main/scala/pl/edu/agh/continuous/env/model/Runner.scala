package pl.edu.agh.continuous.env.model

import io.jvm.uuid.UUID
import pl.edu.agh.continuous.env.common.CellBoundedPosition.PositionExtensions
import pl.edu.agh.continuous.env.common.geometry.{Circle, SweptCircle}
import pl.edu.agh.xinuk.algorithm.Vec2
import pl.edu.agh.xinuk.config.XinukConfig
import pl.edu.agh.xinuk.model.{AgentMessage, Direction, Signal, SphObjectData}

import java.awt.Color
import scala.util.Random

case class SPHRunnerData(pressure: Double = 0, density: Double = 0)

final case class Runner(id: UUID,
                        generation: Long,
                        priority: Long,
                        positionInCell: Vec2,
                        radius: Double,
                        nextStep: Vec2,
                        var path: List[Vec2],
                        speed: Double,  // max speed
                        lastMoveCompletion: Option[MoveCompletion],
                        isStuck: Boolean,
                        color: Color,
                        force: Vec2,  // zeroed before each step,
                        velocity: Vec2,
                        mass: Double,
                        sphData : SphObjectData,
                        tag: String) extends Equals {

  def GenerateSignal(currentTime : Double, cell : ContinuousEnvCell)(implicit config: XinukConfig): Signal = {
    Signal(0, Map(id -> AgentMessage.createNew(
      Vec2(globalCellPosition(config).x + cell.BaseCoordinates().x, globalCellPosition(config).y + cell.BaseCoordinates().y)
      , velocity
      , trueMass, sphData, radius)));
  }
  def body: Circle = Circle(positionInCell, radius)

  def fakeMass: Double = body.area

  def sweptBody: SweptCircle = body.sweep(nextStep)

  def sweptBody(moveCompletion: MoveCompletion): SweptCircle = body.sweep(nextStep * moveCompletion.value)

  def endPosition(moveCompletion: MoveCompletion): Vec2 = positionInCell + (nextStep * moveCompletion.value)

  def endPosition: Vec2 = positionInCell + nextStep
  def globalCellPosition(config: XinukConfig): Vec2 = Vec2(positionInCell.x, config.cellSize - positionInCell.y)

  def maxStepLength(cellSize: Double): Double = cellSize * 0.5 - body.r //this was diameter but should be radius

  def lastActualStep: Option[Vec2] = lastMoveCompletion.map(lmc => nextStep * lmc.value)
  def trueMass: Double = 80 // [kg]
  def maxSpeed: Double = 100 // [cm / s]
  def legForce: Double = trueMass * maxSpeed * maxSpeed / (2 * 100) * 10// reach maxSpeed in 1m = 100cm

  def completeMove(moveCompletion: MoveCompletion): Runner = Runner(
    id,
    generation,
    priority,
    positionInCell + (nextStep * moveCompletion.value),
    radius,
    nextStep,
    path,
    speed,
    Some(moveCompletion),
    moveCompletion.tag.equals("e"),
    color,
    force,
    velocity,
    mass,
    sphData,
    tag
  )

  def withNewPriority(): Runner = Runner(
    id,
    generation,
    Random.nextLong(),
    positionInCell,
    radius,
    nextStep,
    path,
    speed,
    lastMoveCompletion,
    isStuck,
    color,
    force,
    velocity,
    mass,
    sphData,
    tag
  )

  def withReducedNextStep(reducedNextStep: Vec2, dt: Double): Runner =
    {
      if(reducedNextStep.lengthSq > nextStep.lengthSq)
      {
        val step = reducedNextStep.normalized * nextStep.length
        return Runner(id, generation, priority, positionInCell, radius, step, path, speed, None, isStuck, color, force, velocity, mass, sphData, tag)
      }
      // v = s / dt
      var newV = reducedNextStep / dt
      if(newV.lengthSq > velocity.lengthSq)
        newV = newV.normalized * velocity.length;
      return Runner(id, generation, priority, positionInCell, radius, reducedNextStep, path, speed, None, isStuck, color, force, newV, mass, sphData, tag)
    }
  def withNextStep(nextStep: Vec2, newVelocity: Vec2): Runner = Runner(id, generation, priority, positionInCell, radius, nextStep, path, speed, None, isStuck, color, force, newVelocity, mass, sphData, tag)
  def withClearedForce(): Runner = Runner(id, generation, priority, positionInCell, radius, nextStep, path, speed, None, isStuck, color, Vec2.zero, velocity, mass, sphData, tag)
  def withIncreasedForce(increaseForce: Vec2): Runner = Runner(id, generation, priority, positionInCell, radius, nextStep, path, speed, None, isStuck, color, force + increaseForce, velocity, mass, sphData, tag)
  def withNewSphData(sphData : SphObjectData): Runner = Runner(id, generation, priority, positionInCell, radius, nextStep, path, speed, None, isStuck, color, force, velocity, mass, sphData, tag)

  def withAdjustedPosition(cellSize: Double,
                           direction: Direction): Runner = {
    val newPosition = positionInCell.cellBounded(cellSize, false).adjust(direction, false)

    Runner(id, generation, priority, newPosition, radius, nextStep, path, speed, lastMoveCompletion, isStuck, color, force, velocity, mass, sphData, tag)
  }

  def withIncrementedGeneration(): Runner = Runner(
    id,
    generation + 1,
    priority,
    positionInCell,
    radius,
    nextStep,
    path,
    speed,
    lastMoveCompletion,
    isStuck,
    color,
    force,
    velocity,
    mass,
    sphData, tag
  )

  def normalizePosition(cellSize: Double): (Runner, Option[Direction]) = {
    val (newPosition, maybeDirection) = positionInCell.cellBounded(cellSize, false).normalize
    (Runner(id, generation, priority, newPosition, radius, nextStep, path, speed, lastMoveCompletion, isStuck, color, force, velocity, mass, sphData, tag), maybeDirection)
  }

  def inflate(radiusDelta: Double): Runner = Runner(
    id,
    generation,
    priority,
    positionInCell,
    radius + radiusDelta,
    nextStep,
    path,
    speed,
    lastMoveCompletion,
    isStuck,
    color,
    force,
    velocity,
    mass,
    sphData, tag
  )

  override def canEqual(that: Any): Boolean = that.isInstanceOf[Runner]

  override def equals(other: Any): Boolean = other match {
    case that: Runner =>
      (that canEqual this) &&
        id == that.id
    case _ => false
  }

  override def hashCode(): Int = {
    val state = Seq(id)
    state.map(_.hashCode()).foldLeft(0)((a, b) => 31 * a + b)
  }
}

object Runner {
  def apply(id: UUID,
            generation: Long,
            priority: Long,
            position: Vec2,
            radius: Double,
            nextStep: Vec2,
            path: List[Vec2],
            speed: Double,
            lastMoveCompletion: Option[MoveCompletion],
            isStuck: Boolean,
            color: Color,
            force: Vec2,
            velocity: Vec2,
            mass: Double,
            sphData: SphObjectData,
            tag: String): Runner =
    new Runner(
      id,
      generation,
      priority,
      position,
      radius,
      nextStep,
      path,
      speed,
      lastMoveCompletion,
      isStuck = isStuck,
      color,
      force,
      velocity,
      mass,
      sphData, tag)

//  def createNew(position: Vec2,
//                radius: Double,
//                nextStep: Vec2,
//                speed: Double,
//                color: Color): Runner =
//    new Runner(
//      UUID.random,
//      generation = 0,
//      priority = Random.nextLong(),
//      position,
//      radius,
//      nextStep,
//      List.empty,
//      speed,
//      None,
//      isStuck = false,
//      color)

//  def createNew(start: Vec2,
//                end: Vec2,
//                radius: Double,
//                speed: Double,
//                color: Color): Runner =
//    new Runner(
//      UUID.random,
//      generation = 0,
//      priority = Random.nextLong(),
//      start,
//      radius,
//      end - start,
//      List.empty,
//      speed,
//      None,
//      isStuck = false,
//      color)

  def createNew(position: Vec2,
                radius: Double,
                speed: Double,
                color: Color,
                tag: String): Runner =
    new Runner(
      UUID.random,
      generation = 0,
      priority = Random.nextLong(),
      position,
      radius,
      nextStep = Vec2.zero,
      List.empty,
      speed,
      None,
      isStuck = false,
      color,
      Vec2.zero,
      Vec2.zero,
      0,
      SphObjectData(),
      tag)

  def createNewMock(sweptCircle: SweptCircle,
                    speed: Double,
                    color: Color,
                    tag: String): Runner =
    new Runner(
      UUID(0, 0),
      generation = 0,
      priority = 0,
      positionInCell = sweptCircle.start,
      radius = sweptCircle.r,
      nextStep = sweptCircle.line.vector,
      List.empty,
      speed,
      None,
      isStuck = false,
      color,
      Vec2.zero,
      Vec2.zero,
      0,
      SphObjectData(),
      tag)
}