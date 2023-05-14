package pl.edu.agh.continuous.env.algorithm

import pl.edu.agh.continuous.env.common.CellBoundedPosition.PositionExtensions
import pl.edu.agh.continuous.env.common.CollisionAvoidance.RunnerCollisionAvoidanceExtensions
import pl.edu.agh.xinuk.algorithm.MathUtils.DoubleExtensions
import pl.edu.agh.continuous.env.common.ObstacleMapping
import pl.edu.agh.continuous.env.common.RunnerPhysics.RunnerExtensions
import pl.edu.agh.continuous.env.common.ToVec2Conversions.SignalMapConversionExtensions
import pl.edu.agh.continuous.env.common.geometry.Algorithms.LineIntersection
import pl.edu.agh.continuous.env.common.geometry.{ClosestPointInPolygon, Line}
import pl.edu.agh.continuous.env.config.ContinuousEnvConfig
import pl.edu.agh.continuous.env.model.{ContinuousEnvCell, MoveCompletion, Runner, RunnerOccupied}
import pl.edu.agh.xinuk.algorithm.{Plan, PlanCreator, Plans, StateUpdate, Vec2}
import pl.edu.agh.xinuk.model._
import pl.edu.agh.xinuk.model.continuous._
import pl.edu.agh.xinuk.model.grid.GridDirection
import pl.edu.agh.xinuk.model.grid.GridDirection.{BottomLeft, BottomRight, TopLeft, TopRight}

import java.util.{NoSuchElementException, UUID}
import scala.collection.mutable.ListBuffer

object StepAdjustmentCalculator {
  def adjustNextStepToObstaclesAndRunners(runner: Runner,
                                                  neighbourContents: Map[(ContinuousEnvCell, UUID), Direction],
                                                  currentCell: ContinuousEnvCell,
                                                  allReachableRunners: Set[Runner],
                                                  config: ContinuousEnvConfig,
                                                  signalMap: SignalMap): Runner = {
    val inflatedRunners : Seq[Runner] = Helper.inflateRunners(allReachableRunners.toSeq)

    val moveCompletionConsideringObstacles =
      Helper.tryGetMaxMoveCompletionOrMin(() =>
        runner.getMaxMoveCompletionConsideringObstacles(neighbourContents, currentCell, config.cellSize))
    val moveCompletionConsideringRunnersBodies =
      Helper.tryGetMaxMoveCompletionOrMin(() =>
        runner.getMaxMoveCompletionConsideringOtherRunnersBodies(inflatedRunners))
    val maxMoveCompletion = Seq(
      moveCompletionConsideringRunnersBodies,
      moveCompletionConsideringObstacles
    ).minByValue
    if (maxMoveCompletion < MoveCompletion.max()) {
      maxMoveCompletion.normal match {
        case Some(value) =>
          //reportDiagnostics(RunnerStepComponentAdjustmentDiagnostic(runner, maxMoveCompletion, stage))
          val component = runner.nextStep.projectionOn(value) * (MoveCompletion.max().value - maxMoveCompletion.value)
          val twistedNextStep = (runner.nextStep - component).normalized * runner.nextStep.length

          val maxStepLength = runner.maxStepLength(config.cellSize)
          val directionDelta = twistedNextStep.angle - runner.nextStep.angle
          if (directionDelta ~= 0.0) {
            runner
          } else {
            val cosDirectionDelta = math.cos(directionDelta)
            val maxTwistedNextStepLength =
              (maxStepLength * math.sqrt(2) * math.sqrt(1 - cosDirectionDelta)) / (2 - 2 * cosDirectionDelta)

            runner.withReducedNextStep(twistedNextStep.clipLength(lowerBound = 0, upperBound = maxTwistedNextStepLength), config.deltaTime)
          }
        case None =>
          //reportDiagnostics(RunnerEmptyStepComponentAdjustmentDiagnostic(runner, maxMoveCompletion, stage))
          runner
      }
    } else {
      runner
    }
  }

  def limitNextStepToObstacles(runner: Runner,
                                          config: ContinuousEnvConfig,
                                          cell: ContinuousEnvCell,
                                          signalMap: SignalMap): Runner = {
    val myPos = runner.globalCellPosition(config) + cell.BaseCoordinates(config);
    val nextPos = myPos + runner.nextStep
    val (agentMessages, obstacleMessages) = signalMap.toObjectMessagesSplit match {
      case (agents, obstacles) => (agents, obstacles)
      case _ => (List.empty[AgentMessage], List.empty[ObstacleMessage])
    }
    val obstaclePolygons: List[List[Vec2]] = obstacleMessages.map(msg => msg.ToVec2List);
    val wallPoint = ClosestPointInPolygon.closestPointInPolygons(nextPos, obstaclePolygons);

    if((wallPoint - nextPos).length < runner.radius/2)
      return runner.withNextStep(Vec2(0,0), Vec2(0,0));
    //       return runner.withNextStep(- runner.nextStep, -runner.velocity);
    return runner
  }
}
