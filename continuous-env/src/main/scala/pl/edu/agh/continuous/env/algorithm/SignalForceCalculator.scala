package pl.edu.agh.continuous.env.algorithm

import pl.edu.agh.continuous.env.common.CellBoundedPosition.PositionExtensions
import pl.edu.agh.continuous.env.common.CollisionAvoidance.RunnerCollisionAvoidanceExtensions
import pl.edu.agh.xinuk.algorithm.MathUtils.DoubleExtensions
import pl.edu.agh.continuous.env.common.ObstacleMapping
import pl.edu.agh.continuous.env.common.RunnerPhysics.RunnerExtensions
import pl.edu.agh.continuous.env.common.ToVec2Conversions.SignalMapConversionExtensions
import pl.edu.agh.continuous.env.common.geometry.Algorithms.LineIntersection
import pl.edu.agh.continuous.env.common.geometry.Line
import pl.edu.agh.continuous.env.config.ContinuousEnvConfig
import pl.edu.agh.continuous.env.model.{ContinuousEnvCell, MoveCompletion, Runner, RunnerOccupied}
import pl.edu.agh.xinuk.algorithm.{Plan, PlanCreator, Plans, StateUpdate, Vec2}
import pl.edu.agh.xinuk.model._
import pl.edu.agh.xinuk.model.continuous._
import pl.edu.agh.xinuk.model.grid.GridDirection
import pl.edu.agh.xinuk.model.grid.GridDirection.{BottomLeft, BottomRight, TopLeft, TopRight}

import java.util.{NoSuchElementException, UUID}
import scala.collection.mutable.ListBuffer


object SignalForceCalculator {

  def adjustVelocityForRunner(runner: Runner,
                                      signalMap: SignalMap,
                                      cell: ContinuousEnvCell,
                                      neighbourContents: Map[(ContinuousEnvCell, UUID), Direction],
                                      config: ContinuousEnvConfig): Runner = {
    if (runner.path.isEmpty) {
      val force = signalMap.toVec2.normalized
      if (force.length != 0.0) {
        val destinationLine = Line(runner.positionInCell, Vec2(runner.positionInCell.x + force.x * config.cellSize * 2.0,
          runner.positionInCell.y + force.y * config.cellSize * 2.0))
        val destination = Helper.adjustDestination(destinationLine, cell, neighbourContents, config.cellSize)
//        if (cell.graph.isEmpty) {
          runner.path = List(runner.positionInCell, destination)
//        }
//        else {
//          runner.path = Helper.findPath(runner.positionInCell, destination, cell.graph, config.cellSize).toList
//        }
      }
    }

    var force = Vec2.zero
    if (runner.path.nonEmpty) {
      var target = Vec2.zero
      try {
        target = Helper.findNextStep(runner.path, runner.positionInCell, cell, neighbourContents, config.cellSize)
      }
      catch {
        case _: NoSuchElementException => runner.path = Helper.findPath(runner.positionInCell, runner.path.last, cell.graph, config.cellSize).toList
          target = Helper.findNextStep(runner.path, runner.positionInCell, cell, neighbourContents, config.cellSize)
      }
      val movementVector = Line(runner.positionInCell, target)
      force = movementVector.end - movementVector.start
    }
    var forceFactor = runner.legForce
    val adjustedRunner = runner //TODO acceleration must be increased by cellsize^2
      .withClearedForce()
      .withNewPriority()
      .withIncrementedGeneration()
      .withIncreasedForce(force.normalized * forceFactor)
    //      .withAppliedForceConsideringLastStep(
    //        nextStep,
    //        config.personUnitAcceleration,
    //        config.personMinStepLength,
    //        config.cellSize)

    Helper.reportPossibleFulfillmentDiagnostics(adjustedRunner, config.cellSize)
    return adjustedRunner
  }
}
