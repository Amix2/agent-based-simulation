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

object Helper {
  def reportPossibleFulfillmentDiagnostics(runner: Runner,
                                                   cellSize: Double): Unit = {
    val maxStepLength = runner.maxStepLength(cellSize)

    /*reportDiagnostics(RunnerPossibleStepFulfillment(
      runner = runner,
      maxStepLength = maxStepLength,
      normalizedStepLength = runner.nextStep.length
    ))*/
  }

  def adjustDestination(destinationLine: Line,
                                cell: ContinuousEnvCell,
                                neighbourContents: Map[(ContinuousEnvCell, UUID), Direction],
                                cellSize: Int): Vec2 = {
    //val destination = limitLineToNeighbourObstacles(destinationLine, neighbourContents, cellSize)
    val destination = destinationLine
    var targetSegmentVertice = Vec2.zero
    if (destination.end.x <= cellSize.doubleValue && destination.end.x >= 0.0 &&
      destination.end.y <= cellSize.doubleValue && destination.end.y >= 0.0) {
      val targetSegment = cell.cardinalSegments.minBy(segmentEntry =>
        Math.min(Line(destination.end, segmentEntry._1.start).length,
          Line(destination.end, segmentEntry._1.end).length))._1

      if (Line(destination.end, targetSegment.start).length <
        Line(destination.end, targetSegment.end).length) {
        targetSegmentVertice = targetSegment.start
      }
      else {
        targetSegmentVertice = targetSegment.end
      }
      val destinationDir = getDirectionForCoords(targetSegment.center, cellSize.doubleValue)
      val neighbour = getNeighbourForLine(neighbourContents, targetSegment, cellSize)
      if (neighbour.graph.nonEmpty) {
        return neighbour.graph
          .map(graphVertice => graphVertice._1.cellBounded(cellSize, true).adjust(destinationDir, true))
          .minBy(graphVertice => Line(graphVertice, targetSegmentVertice).length)
      }
      return destination.start + Vec2(destination.start, targetSegmentVertice) * 1.5
    }

    val destinationDir = getDirectionForCoords(destination.end, cellSize.doubleValue)
    var neighbour: ContinuousEnvCell = cell
    var targetSegment = (Line(Vec2.zero, Vec2.zero), Vec2.zero)
    var sendToDiagonal: Boolean = false
    if (destinationDir.isDiagonal) {
      neighbour = neighbourContents.filter(cell => cell._2.equals(destinationDir))
        .map(cell => cell._1._1)
        .head
      //we search for segment crossed to reach diagonal neighbour
      //if potentialTargetSegments is empty, it means the original destination vector leads outside of diagonal neighbour's cellOutline
      //so we must give up on it and search for a segment which was crossed first to reach one of cardinal neighbours
      //and will send agent to respective cardinal neighbour rather than non-existent diagonal one
      val adjustedNeighbourSegments = neighbour.cardinalSegments.keys
        .map(segment => Line(segment.start.cellBounded(cellSize, true).adjust(destinationDir, true),
          segment.end.cellBounded(cellSize, true).adjust(destinationDir, true)))
      val potentialTargetSegments = crossedNeighbourSegments(destination, adjustedNeighbourSegments)
      if (potentialTargetSegments.nonEmpty) {
        targetSegment = potentialTargetSegments
          .minBy(crossedSegment => Line(crossedSegment._2, destination.end).length)
        //in the line above we search crossed segments for which point of crossing is nearest to the position of agent for whom we adjust destination
        sendToDiagonal = true
        if (neighbour.graph.nonEmpty) {
          var point = Vec2.zero
          if (destinationDir.equals(BottomLeft)) point = Vec2(0, 0)
          if (destinationDir.equals(BottomRight)) point = Vec2(cellSize, 0)
          if (destinationDir.equals(TopLeft)) point = Vec2(0, cellSize)
          if (destinationDir.equals(TopRight)) point = Vec2(cellSize, cellSize)
          return neighbour.graph
            .map(graphVertice => graphVertice._1.cellBounded(cellSize, true).adjust(destinationDir, true))
            .minBy(graphVertice => Line(graphVertice, point).length)
        }
      }
    }
    if (!sendToDiagonal) {
      targetSegment = crossedNeighbourSegments(destination, cell.cardinalSegments.keys)
        .minBy(crossedSegment => Line(crossedSegment._2, destination.start).length)
      neighbour = getNeighbourForLine(neighbourContents, targetSegment._1, cellSize)
    }

    if (neighbour.graph.nonEmpty) {
      return neighbour.graph
        .map(graphVertice => graphVertice._1.cellBounded(cellSize, true).adjust(destinationDir, true))
        .minBy(graphVertice => Line(graphVertice, targetSegment._2).length)
    }
    destination.end
  }

  def getDirectionForCoords(coords: Vec2, cellSize: Double): GridDirection = {
    if (coords.x <= 0.0) {
      if (coords.y <= 0.0) {
        return GridDirection.BottomLeft
      }
      if (coords.y >= cellSize) {
        return GridDirection.TopLeft
      }
      return GridDirection.Left
    }
    if (coords.x >= cellSize) {
      if (coords.y <= 0.0) {
        return GridDirection.BottomRight
      }
      if (coords.y >= cellSize) {
        return GridDirection.TopRight
      }
      return GridDirection.Right
    }
    if (coords.y <= 0.0) {
      return GridDirection.Bottom
    }
    GridDirection.Top
  }

  def getNeighbourForLine(neighbourContents: Map[(ContinuousEnvCell, UUID), Direction],
                                  segment: Line,
                                  cellSize: Int): ContinuousEnvCell = {
    /*.map(cell => cell._1._1.cardinalSegments.start.cellBounded(cellSize, true).adjust(destinationDir, true),
      segment.end.cellBounded(cellSize, true).adjust(destinationDir, true))*/
    val dirNeighbours: Map[(ContinuousEnvCell, UUID), Iterable[Line]] = neighbourContents
      .filter(cell => cell._2.asInstanceOf[GridDirection].isCardinal)
      .map(cell => (cell._1, cell._1._1.cardinalSegments
        .map(segment => Line(segment._1.start.cellBounded(cellSize, true).adjust(cell._2, true),
          segment._1.end.cellBounded(cellSize, true).adjust(cell._2, true)))))
    dirNeighbours
      .filter(neighbour => neighbour._2.toList.contains(segment))
      .keys.head._1
    /*val dirNeighbours: List[ContinuousEnvCell] = neighbourContents
      .filter(cell => cell._2.asInstanceOf[GridDirection].isCardinal)
      .map(cell => cell._1._1)
      .toList
    dirNeighbours
      .filter(neighbour => neighbour.cardinalSegments.exists(entry => entry._1.equals(segment)))
      .head*/
  }

  def crossedNeighbourSegments(crossingLine: Line,
                                       segments: Iterable[Line]): List[(Line, Vec2)] = {
    segments
      .map(segment => (segment, segment.intersect(crossingLine)))
      .filter(intersection => intersection._2.nonEmpty)
      .filter(intersection => intersection._2.get.onLine1 && intersection._2.get.onLine2)
      .map(intersection => (intersection._1, intersection._2.get.pos))
      .toList
  }

  def findPath(start: Vec2, end: Vec2, graph: Map[Vec2, Set[Vec2]], cellSize: Int): ListBuffer[Vec2] = {
    val closestStart: Vec2 = graph.minBy(v => Line(v._1, start).length)._1
    val closestEnd: Vec2 = graph.minBy(v => Line(v._1, end).length)._1
    var adjustedGraph: Map[Vec2, Set[Vec2]] = graph
    adjustedGraph = adjustedGraph + (start -> Set(closestStart))
    adjustedGraph = adjustedGraph.updatedWith(closestStart)({ case Some(verticeNeighbours) => Some(verticeNeighbours ++ Set(start))
    case _ => throw new RuntimeException("could not update graph")
    })
    adjustedGraph = adjustedGraph + (end -> Set(closestEnd))
    adjustedGraph = adjustedGraph.updatedWith(closestEnd)({ case Some(verticeNeighbours) => Some(verticeNeighbours ++ Set(end))
    case _ => throw new RuntimeException("could not update graph")
    })

    val discoveredNodes: scala.collection.mutable.Set[Vec2] = scala.collection.mutable.Set(start)
    val cameFrom: scala.collection.mutable.Map[Vec2, Vec2] = scala.collection.mutable.Map.empty
    val discoveredPathCost: scala.collection.mutable.Map[Vec2, Double] = scala.collection.mutable.Map.empty
    discoveredPathCost.put(start, 0.0)
    val totalPathEstimatedCost: scala.collection.mutable.Map[Vec2, Double] = scala.collection.mutable.Map.empty
    totalPathEstimatedCost.put(start, Line(start, end).length) //heuristic function is simply distance

    while (discoveredNodes.nonEmpty) {
      val currentNode: Vec2 = discoveredNodes.minBy(node => totalPathEstimatedCost.get(node).orElse(Some(cellSize.doubleValue * 10.0)))
      if (currentNode.x == end.x && currentNode.y == end.y) {
        return reconstructPath(cameFrom, currentNode)
      }
      discoveredNodes -= currentNode
      adjustedGraph(currentNode).foreach(neighbourNode => {
        val currentDistance: Double = discoveredPathCost(currentNode) + Line(currentNode, neighbourNode).length
        if (currentDistance < discoveredPathCost.get(neighbourNode).orElse(Some(cellSize.doubleValue * 10.0)).get) {
          cameFrom.put(neighbourNode, currentNode)
          discoveredPathCost.put(neighbourNode, currentDistance)
          totalPathEstimatedCost.put(neighbourNode, currentDistance + Line(neighbourNode, end).length)
          discoveredNodes += neighbourNode
        }
      })
    }
    throw new RuntimeException("Failed to find path for graph: " + adjustedGraph.toString())
  }

  def findNextStep(path: List[Vec2],
                           startingPosition: Vec2,
                           cell: ContinuousEnvCell,
                           neighbourContents: Map[(ContinuousEnvCell, UUID), Direction],
                           cellSize: Int): Vec2 = {
    val lines = ObstacleMapping.NeighborContentsExtensions(neighbourContents + ((cell, UUID.randomUUID()) -> null))
      .mapToObstacleLines(cellSize).toList
    var result = startingPosition
    var flag = false
    path.reverse.foreach(vertice => {
      val intersections: List[(Line, Option[LineIntersection])] = lines
        .map(line => (line, line.intersect(Line(startingPosition, vertice))))
        .filter(intersection => intersection._2.nonEmpty)
        .filter(intersection => intersection._2.get.onLine1 && intersection._2.get.onLine2)
      if (intersections.isEmpty && !flag) {
        result = vertice
        flag = true
      }
    })
    result
    /*path.findLast(vertice => !ObstacleMapping.NeighborContentsExtensions(neighbourContents + ((cell, UUID.randomUUID()) -> null))
      .mapToObstacleLines(cellSize)
      .map(line => (line, line.intersect(Line(startingPosition, vertice))))
      .filter(intersection => intersection._2.nonEmpty)
      .exists(intersection => intersection._2.get.onLine1 && intersection._2.get.onLine2))
      .get*/
    //replaced head with get
  }

  def reconstructPath(cameFrom: scala.collection.mutable.Map[Vec2, Vec2],
                              current: Vec2): ListBuffer[Vec2] = {
    val path: ListBuffer[Vec2] = ListBuffer(current)
    var currentNode: Vec2 = current
    while (cameFrom.contains(currentNode)) {
      currentNode = cameFrom.get(currentNode).orNull
      path.prepend(currentNode)
    }
    path
  }

  def inflateRunners(runners: Seq[Runner]): Seq[Runner] = runners.map(_.inflate(0.01))

  def tryGetMaxMoveCompletionOrMin(moveCompletionComputationAction: () => MoveCompletion): MoveCompletion =
    try {
      moveCompletionComputationAction()
    } catch {
      case e: Exception =>
        e.printStackTrace()
        MoveCompletion.min(tag = "e")
    }
}
