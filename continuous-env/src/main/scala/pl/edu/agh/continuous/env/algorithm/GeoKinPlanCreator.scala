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

final case class GeoKinPlanCreator() extends PlanCreator[ContinuousEnvConfig] {

  override def createPlans(iteration: Long,
                           cellId: CellId,
                           cellState: CellState,
                           neighbourhoodState: NeighbourhoodState)
                          (implicit config: ContinuousEnvConfig): (Plans, GeoKinMetrics) = {
    cellState.contents match {
      case continuousEnvCell: ContinuousEnvCell =>
        if (continuousEnvCell.runners.nonEmpty) {
          var neighboursMap: Map[GridDirection, Iterable[(ContinuousEnvCell, UUID)]] = neighbourhoodState.diagonalNeighbourhoodState
            .filter { case (_, cellState) => !cellState.equals(CellState.empty()) }
            .map(cell => (cell._1, Iterable.single((cell._2.contents.asInstanceOf[ContinuousEnvCell]), UUID.randomUUID())))
          neighboursMap ++= neighbourhoodState.cardinalNeighbourhoodState
            .map(state => (state._1, mapBoundaryStateToCells2(state._2)))
          val reverseNeighboursMap = for ((k, v) <- neighboursMap) yield (v, k)
          val flattenedNeighboursMap: Map[(ContinuousEnvCell, UUID), GridDirection] = reverseNeighboursMap.flatMap { case (k, v) => k.map(_ -> v) }

          val gridCellId = cellId.asInstanceOf[GridMultiCellId]
          val allReachableRunners = getAllReachableRunners(continuousEnvCell, flattenedNeighboursMap, config)
          //reportDiagnostics(AllReachableRunnersDiagnostic(gridCellId, allReachableRunners, ro.runners))
          assertNoOverlaps(iteration, gridCellId, allReachableRunners)

          if (iteration % 2 == 0)
          { // przesuwanie runnerów
            (moveRunnersFromCell(gridCellId, continuousEnvCell, flattenedNeighboursMap, neighbourhoodState, allReachableRunners, config),
              GeoKinMetrics.empty)
            //(Plans.empty, GeoKinMetrics.empty)

          } else {
            (adjustVelocityInCell(continuousEnvCell, cellState.signalMap, flattenedNeighboursMap, allReachableRunners.toSet, config),
              GeoKinMetrics.empty)
          }
        }
        else {
          (Plans.empty, GeoKinMetrics.empty)
        }
      case _ => (Plans.empty, GeoKinMetrics.empty)
    }
  }

  private def mapBoundaryStateToCells2(state: BoundaryState): Iterable[(ContinuousEnvCell, UUID)] = {
    state.boundaryStates.values
      .map(cell => (cell.contents.asInstanceOf[ContinuousEnvCell], UUID.randomUUID()))
  }

  private def mapBoundaryStateToCells(state: BoundaryState): Iterable[ContinuousEnvCell] = {
    state.boundaryStates.values
      .map(cell => cell.contents.asInstanceOf[ContinuousEnvCell])
  }

  private def assertNoOverlaps(iteration: Long,
                               cellId: GridMultiCellId,
                               allReachableRunners: Array[Runner]): Unit = {
    for {
      (r1, r1idx) <- allReachableRunners.zipWithIndex
      (r2, r2idx) <- allReachableRunners.zipWithIndex
      if r1idx < r2idx
    } {
      val r1Body = r1.body
      val r2Body = r2.body
      val bodiesIntersect = r1Body.intersects(r2Body)
      if (bodiesIntersect) {
        //reportDiagnostics(RunnerInCollisionDiagnostic(r1))
        //reportDiagnostics(RunnerInCollisionDiagnostic(r2))
       // throw new IllegalStateException(s"Iteration[$iteration] Cell(${cellId.x}, ${cellId.y}) " +
       //   s"Runners $r1 and $r2 collide with each other!")
        println(s"Iteration[$iteration] Cell(${cellId.x}, ${cellId.y}) " +
          s"Runners $r1 and $r2 collide with each other!")
      }
    }
  }


  private def adjustVelocityInCell(cell: ContinuousEnvCell,
                                   signalMap: SignalMap,
                                   neighbourContents: Map[(ContinuousEnvCell, UUID), Direction],
                                   allReachableRunners: Set[Runner],
                                   config: ContinuousEnvConfig): Plans = {

    val runnersWithAdjustedVelocity = cell.runners.toSet
      .map(runner => beforeStepForRunner( // zero force
          runner,
          signalMap,
          cell,
          neighbourContents,
          config))
      .map(runner => SignalForceCalculator.adjustVelocityForRunner( // force
        runner,
        signalMap,
        cell,
        neighbourContents,
        config))
//      .map(runner => adjustSocialForceForRunner( // force
//        runner,
//        signalMap,
//        cell,
//        neighbourContents,
//        config))
      .map(runner => SphForceCalculator.adjustSphForRunner(  // sph force
        runner,
        signalMap,
        cell,
        neighbourContents,
        config))
      .map(runner => endStepRunnerUpdate( // doesnt matter
        runner,
        signalMap,
        cell,
        neighbourContents,
        config))
      .map(runner => applyForceForRunner( // force -> velocity, next step
        runner,
        config))
      .map(runner => ORCAVelocityCalculator.adjustVelocityForRunner( // ORCA collision avoidance
        runner,
        signalMap,
        cell,
        neighbourContents,
        config))
      //.map(runner => runner.withNextStep(Vec2(0,0),Vec2(0,0)))
//      .map(runner => StepAdjustmentCalculator.adjustNextStepToObstaclesAndRunners( // nextStep
//        runner,
//        neighbourContents,
//        cell,
//        allReachableRunners - runner,
//        config,
//        signalMap))
//    .map(runner => StepAdjustmentCalculator.limitNextStepToObstacles( // nextStep
//      runner,
//      config,
//      cell,
//      signalMap))
//      .map(runner => RepellingForceCalculator.adjustNextStepWithRepellingForceFromObstacles(  // force ?
//        runner,
//        cell,
//        neighbourContents,
//        config))
//      .map(runner => applyForceForRunner( // force -> velocity, next step
//        runner,
//        config))
//      .map(runner => StepAdjustmentCalculator.adjustNextStepToObstaclesAndRunners(
//        runner,
//        neighbourContents,
//        cell,
//        allReachableRunners - runner,
//        config,
//        signalMap))
    new Plans(Map.empty, Seq(Plan(StateUpdate(RunnerOccupied(cell.generation + 1, runnersWithAdjustedVelocity)))))
  }

  def beforeStepForRunner(runner: Runner,
                                 signalMap: SignalMap,
                                 cell: ContinuousEnvCell,
                                 neighbourContents: Map[(ContinuousEnvCell, UUID), Direction],
                                 config: ContinuousEnvConfig): Runner = {

    return runner.withClearedForce();

  }
  def adjustSocialForceForRunner(runner: Runner,
                                 signalMap: SignalMap,
                                 cell: ContinuousEnvCell,
                                 neighbourContents: Map[(ContinuousEnvCell, UUID), Direction],
                                 config: ContinuousEnvConfig): Runner = {
    var force = Vec2(0,0);
    val strength = runner.legForce;
    if(runner.tag.contains("R"))
      force += Vec2(0,-strength);
    if (runner.tag.contains("L"))
      force += Vec2(0, strength);
    if (runner.tag.contains("U"))
      force += Vec2(-strength, 0);
    if (runner.tag.contains("D"))
      force += Vec2(strength, 0);
    return runner.withIncreasedForce(force);


//
//    if(runner.tag == "B")
//         return runner.withIncreasedForce(Vec2(0,10000));
//    if(runner.tag == "A")
//      return runner.withIncreasedForce(Vec2(0,-10000));
//
//    if (runner.tag == "C")
//      return runner.withIncreasedForce(Vec2(10000, 0));
//    if (runner.tag == "D")
//      return runner.withIncreasedForce(Vec2(-10000, 0));
//
//    var agentGeomCenter = Vec2(0, 0);
//    var count = 0;
//
//    val (agentMessages, obstacleMessages) = signalMap.toObjectMessagesSplit match {
//      case (agents, obstacles) => (agents, obstacles)
//      case _ => (List.empty[AgentMessage], List.empty[ObstacleMessage])
//    }
//
//    agentMessages.foreach({ case (sig) => {
//      agentGeomCenter = agentGeomCenter + sig.pos
//      count += 1;
//    }})
//
//    if(count == 0)
//      return runner;
//
//    agentGeomCenter = agentGeomCenter / count;
//   // agentGeomCenter = Vec2(835.0,815.0);  // 635.0,515.0
//    var runnerGlobalPos = runner.globalCellPosition(config) + cell.BaseCoordinates(config);
//    var dir = agentGeomCenter - runnerGlobalPos;
//    if(dir.lengthSq == 0)
//      return runner;
//    dir = dir.normalized
//
//    var swappedDir = Vec2(dir.y, dir.x);
//    var fixedDir = Vec2(dir.x, -dir.y);
//    var force = fixedDir*5000.1;
//    //force = Vec2(0.0,0);
//    var out = runner.withIncreasedForce(force)
//    return out;

  }

  def applyForceForRunner(runner: Runner, config: ContinuousEnvConfig): Runner = {
    var dt = config.deltaTime;
    // a = F/m
    var a = runner.force / runner.trueMass
    // v += a * dt
    var v = runner.velocity + a * dt;
    var maxSpeed = runner.maxSpeed;
    if(v.length > maxSpeed)
    {
      var velOver = v.length - maxSpeed
      var neededSlowForce = velOver * runner.trueMass / dt;
      var possibleSlowForce = runner.legForce
      if(neededSlowForce < possibleSlowForce)
        v = v.normalized * maxSpeed;
      else
      {
        var slowAcc = possibleSlowForce / runner.trueMass
        v += -v.normalized * slowAcc * dt;
      }
    }

    // s = v * dt;
    var nextStep = v * dt;

    runner.withNextStep(nextStep, v);
  }


  def endStepRunnerUpdate(runner: Runner,
                                 signalMap: SignalMap,
                                 cell: ContinuousEnvCell,
                                 neighbourContents: Map[(ContinuousEnvCell, UUID), Direction],
                                 config: ContinuousEnvConfig): Runner = {
    return runner.withIncrementedGeneration();
  }

  private def moveRunnersFromCell(gridCellId: CellId,
                                  cell: ContinuousEnvCell,
                                  neighbourContents: Map[(ContinuousEnvCell, UUID), GridDirection],
                                  state: NeighbourhoodState,
                                  allReachableRunners: Array[Runner],
                                  config: ContinuousEnvConfig): Plans = {
    val plansGroupedByCellId: Map[CellId, Set[Plan]] = cell.runners.toSet
      .map(runner => moveRunner(
        cell.generation,
        runner,
        cell,
        gridCellId,
        allReachableRunners.toSet - runner,
        neighbourContents,
        state,
        config))
      .groupMap {
        case (cellId, _) => cellId
      } {
        case (_, plan) => plan
      }

    val localPlans: Seq[Plan] = plansGroupedByCellId
      .getOrElse(gridCellId, Seq(Plan(StateUpdate(RunnerOccupied(cell.generation + 1, Set())))))
      .toSeq
    val outwardPlans: Map[CellId, Seq[Plan]] = plansGroupedByCellId
      .filterNot {
        case (cellId, _) => cellId.equals(gridCellId)
      }
      .map {
        case (cellId, value) => (cellId, value.toSeq)
      }

    //reportDiagnostics(RunnerChangeCellDiagnostic(gridCellId, outwardPlans))

    new Plans(outwardPlans, localPlans)
  }

  private def moveRunner(generation: Long,
                         runner: Runner,
                         cell: ContinuousEnvCell,
                         cellId: CellId,
                         allReachableRunners: Set[Runner],
                         neighbourContents: Map[(ContinuousEnvCell, UUID), Direction],
                         state: NeighbourhoodState,
                         config: ContinuousEnvConfig): (CellId, Plan) = {
    if (runner.nextStep.lengthSq > 0)
    {
      //      val onTheRightSideRunners = filterRunnersBySide(runner, allReachableRunners.toSeq)
      val inflatedRunners = Helper.inflateRunners(allReachableRunners.toSeq)

      val moveCompletion = Helper.tryGetMaxMoveCompletionOrMin(() => runner.getMaxMoveCompletion(
        inflatedRunners,
        neighbourContents,
        cell,
        config.cellSize))
      //reportDiagnostics(RunnerMoveCompletionDiagnostic(runner, moveCompletion))

      val safeMoveCompletion = moveCompletion.safeRounded
      //reportDiagnostics(RunnerSafeRoundedMoveCompletionDiagnostic(runner, safeMoveCompletion))

      val movedRunner = runner.completeMove(safeMoveCompletion)
      val (normalizedRunner, destinationDirection) = movedRunner.normalizePosition(config.cellSize)
      if (destinationDirection.isEmpty)
      {
        (cellId, Plan(StateUpdate(RunnerOccupied(generation + 1, Set(normalizedRunner)))))
      }
      else
      {
        normalizedRunner.path = List.empty
        (getTargetNeighbour(cell.neighbourhood, cell, destinationDirection.get.asInstanceOf[GridDirection], Line(runner.positionInCell, movedRunner.positionInCell)),
          Plan(StateUpdate(RunnerOccupied(generation + 1, Set(normalizedRunner)))))
      }
      //musi być runner occupied, żeby móc kilkukrotnie przy resolve plans wpisać dodatkowych agentów do tej samej komórki tak żeby się wszyscy zapisali
    } else {
      (cellId, Plan(StateUpdate(RunnerOccupied(generation + 1, Set(runner)))))
    }
  }

  private def getTargetNeighbour(neighbourhood: Neighbourhood,
                                 cell: ContinuousEnvCell,
                                 gridDirection: GridDirection,
                                 runnerStep: Line): GridMultiCellId = {
    if (gridDirection.isDiagonal) {
      return neighbourhood.diagonalNeighbourhood
        .filter {
          case (direction, _) => direction.equals(gridDirection)
        }
        .map {
          case (_, cellId) => cellId
        }
        .headOption.orNull
    }
    getCardinalTargetNeighbour(runnerStep, cell.cardinalSegments)
  }
  def getRandomItem[K, V](map: Map[K, V]): Option[(K, V)] = {
    val list = map.toList
    if (list.isEmpty) None
    else {
      val randomIndex = scala.util.Random.nextInt(list.length)
      Some(list(randomIndex))
    }
  }
  private def getCardinalTargetNeighbour(runnerStep: Line,
                                         cardinalNeighbourLineMap: Map[Line, GridMultiCellId]): GridMultiCellId = {
      var target = cardinalNeighbourLineMap
        .map(neighbourEntry => (neighbourEntry._1.intersect(runnerStep), neighbourEntry._2))
        .filter(intersectionEntry => intersectionEntry._1.nonEmpty)
        .filter(intersectionEntry => intersectionEntry._1.get.onLine1 && intersectionEntry._1.get.onLine2)

      if(target.isEmpty)
          return getRandomItem(cardinalNeighbourLineMap).get._2 // yolo
      target.head._2
  }

  private def getAllReachableRunners(cell: ContinuousEnvCell,
                                     neighbours: Map[(ContinuousEnvCell, UUID), GridDirection],
                                     config: ContinuousEnvConfig): Array[Runner] = {
    var result: Array[Runner] = cell.runners
    neighbours
      .filter(neighbour => neighbour._1._1.runners.nonEmpty)
      .foreach(neighbour => result ++= neighbour._1._1.runners.map(x => x.withAdjustedPosition(config.cellSize, neighbour._2)))
    result
  }

}
