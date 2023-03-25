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

object RepellingForceCalculator {
  def adjustNextStepWithRepellingForceFromObstacles(runner: Runner,
                                                            currentCell: ContinuousEnvCell,
                                                            neighbourContents: Map[(ContinuousEnvCell, UUID), Direction],
                                                            config: ContinuousEnvConfig): Runner = {
    /*val toBottom = Vec2(1, 0)
    val toTop = -toBottom
    val toLeft = Vec2(0, 1)
    val toRight = -toLeft*/
    val toBottom = Vec2(0, -1)
    val toTop = -toBottom
    val toLeft = Vec2(-1, 0)
    val toRight = -toLeft

    val cells: Map[(ContinuousEnvCell, UUID), Direction] = neighbourContents + ((currentCell, UUID.randomUUID()) -> null)
    val obstacleSegments: Iterable[Line] = ObstacleMapping.NeighborContentsExtensions(cells)
      .mapToObstacleLines(config.cellSize)
      .filter(line => line.segmentDistance(runner.positionInCell + runner.nextStep) < 0.1)
    if (obstacleSegments.isEmpty) {
      runner
    }
    else {
      val repellingForceBase = obstacleSegments.flatMap(segment => getObstacleDirection(segment, runner.positionInCell + runner.nextStep, 0.1) match {
        case GridDirection.Top => Some(toBottom)
        case GridDirection.Right => Some(toLeft)
        case GridDirection.Bottom => Some(toTop)
        case GridDirection.Left => Some(toRight)
        case _ => throw new UnsupportedOperationException("Unknown direction")
      }).fold(Vec2.zero)((v1: Vec2, v2: Vec2) => v1 + v2)

      val repellingForceFactor = 1.0
      val repellingForce = repellingForceBase * repellingForceFactor

      runner.withIncreasedForce(repellingForce)
      //      runner.withAppliedForceConstrainedNoMin(
      //        repellingForce,
      //        config.personUnitAcceleration,
      //        config.cellSize)
    }
  }
  def getObstacleDirection(line: Line, position: Vec2, minDistance: Double): GridDirection = {
    val hLine: Line = Line(Vec2(position.x - minDistance * 2, position.y), Vec2(position.x + minDistance * 2, position.y))
    val vLine: Line = Line(Vec2(position.x, position.y - minDistance * 2), Vec2(position.x, position.y + minDistance * 2))

    val intersections: Seq[Option[LineIntersection]] = Seq(line.intersect(hLine), line.intersect(vLine))
    val nearestPoint: LineIntersection = intersections.filter(intersection => intersection.nonEmpty)
      .map(intersection => intersection.get)
      .find(intersection => intersection.onLine1 && intersection.onLine2)
      .orNull

    // fixme null ptr exception - probably it was just bad initial agent positioning/config
    if (nearestPoint.pos.x - position.x > 0) {
      GridDirection.Right
    }
    else if (nearestPoint.pos.x - position.x < 0) {
      GridDirection.Left
    }
    else if (nearestPoint.pos.y - position.y > 0) {
      GridDirection.Top
    }
    else {
      GridDirection.Bottom
    }
  }
}
