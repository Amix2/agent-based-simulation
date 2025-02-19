package pl.edu.agh.continuous.env.algorithm

import org.locationtech.jts.geom.{Coordinate, Envelope, GeometryCollection, GeometryFactory}
import org.locationtech.jts.operation.buffer.BufferParameters
import org.locationtech.jts.triangulate.VoronoiDiagramBuilder
import org.slf4j.Logger
import pl.edu.agh.continuous.env.common.geometry.Line
import pl.edu.agh.continuous.env.config.ContinuousEnvConfig
import pl.edu.agh.continuous.env.model.continuous.CellOutline
import pl.edu.agh.continuous.env.model.{ContinuousEnvCell, Runner}
import pl.edu.agh.xinuk.algorithm.{Vec2, Vec2Int, WorldCreator}
import pl.edu.agh.xinuk.config.{Obstacle, RunnerDefinition}
import pl.edu.agh.xinuk.model.continuous._
import pl.edu.agh.xinuk.model.grid.GridDirection.{Bottom, BottomLeft, BottomRight, Left, Right, Top, TopLeft, TopRight}
import pl.edu.agh.xinuk.model.grid.{GridCellId, GridDirection, GridWorldBuilder}
import pl.edu.agh.xinuk.model.{CellState, Signal, WorldBuilder}

import java.awt.geom.Area
import java.awt.{Color, Polygon}
import java.util.UUID
import scala.collection.immutable.List
import scala.collection.mutable
import scala.collection.mutable.{Map => MutableMap}
import scala.jdk.CollectionConverters._
import scala.math.max
import scala.swing.Rectangle


object ContinuousEnvWorldCreator extends WorldCreator[ContinuousEnvConfig] {

  var logger: Logger = _

  override def prepareWorld()(implicit config: ContinuousEnvConfig): WorldBuilder = {
    val worldBuilder = GridWorldBuilder().withGridConnections()

    val multiCellIdMap: MutableMap[GridCellId, Int] = MutableMap.empty
    val cellOutlineMap: MutableMap[GridMultiCellId, CellOutline] = MutableMap.empty.withDefault(_ => CellOutline.default())
    val cellQueue: mutable.Queue[GridMultiCellId] = mutable.Queue.empty
    val finalCellQueue: mutable.Queue[GridMultiCellId] = mutable.Queue.empty

    for {
      x <- 0 until config.worldWidth
      y <- 0 until config.worldHeight
    } {
      cellQueue.enqueue(GridMultiCellId(x, y, 0))
      multiCellIdMap(GridCellId(x, y)) = 0
    }

    val obstacles = bufferObstacles(config.obstacles)
    val obstaclesWithMapBorder = bufferObstacles(config.obstacles ++ GetGlobalBorderObstacles())

    while (cellQueue.nonEmpty) {
      val gridMultiCellId: GridMultiCellId = cellQueue.dequeue()
      val x = gridMultiCellId.x
      val y = gridMultiCellId.y
      val continuousEnvCell: ContinuousEnvCell = if (x == 23 && y >=11 && y <=13) {
        //ContinuousEnvCell(config.initialSignal, gridMultiCellId)
        ContinuousEnvCell(Signal.zero, gridMultiCellId)
      } else {
        ContinuousEnvCell(Signal.zero, gridMultiCellId)
      }

      continuousEnvCell.neighbourhood = worldBuilder.getExistingNeighbourhood(gridMultiCellId)
      continuousEnvCell.cellOutline = cellOutlineMap(gridMultiCellId)

      continuousEnvCell.globalObstacles = FixGlobalObstacles(getOverlappingGlobalObstacles(continuousEnvCell, obstaclesWithMapBorder, x, y)).toArray
      val overlappingObstacles = getOverlappingObstacles(continuousEnvCell, obstacles, x, y)

      if (overlappingObstacles.nonEmpty) {
        var obstaclesGroups: Array[Array[Obstacle]] = Array()
        var cellDivided = false

        for (obstacle <- overlappingObstacles) {
          var obstaclesToMerge = Array(obstacle)
          var newObstaclesGroups: Array[Array[Obstacle]] = Array()
          for (existingObstacleGroup <- obstaclesGroups) {
            if (overlapsWithAny(obstacle, existingObstacleGroup)) {
              obstaclesToMerge = obstaclesToMerge ++ existingObstacleGroup
            } else {
              newObstaclesGroups = newObstaclesGroups :+ existingObstacleGroup
            }
          }
          newObstaclesGroups = newObstaclesGroups :+ obstaclesToMerge
          obstaclesGroups = newObstaclesGroups

          if (isObstaclesGroupDividingCell(continuousEnvCell.cellOutline, obstaclesToMerge, x, y)) {
            val newCells = divideCells(continuousEnvCell, obstaclesToMerge, x, y)
              .filter(cell => !hasEmptyNeighbourhood(cell))
              .filter(cell => !hasSameNeighbourhood(cell, continuousEnvCell))

            if (newCells.nonEmpty) {
              cellDivided = true

              val currentId = multiCellIdMap(GridCellId(x, y))
              val currentGridMultiCellId = GridMultiCellId(x, y, currentId)
              val newCellNeighbourhoodMap: MutableMap[GridMultiCellId, Neighbourhood] = MutableMap.empty
              for (i <- newCells.indices) {
                val nextId = currentId + i + 1
                val newCellGridMultiCellId = GridMultiCellId(x, y, nextId)
                newCellNeighbourhoodMap(newCellGridMultiCellId) = newCells(i).neighbourhood
                cellOutlineMap(newCellGridMultiCellId) = newCells(i).cellOutline
                cellQueue.enqueue(newCellGridMultiCellId)
              }
              worldBuilder.updateNeighbourhoodAfterDividingCell(currentGridMultiCellId, Map.from(newCellNeighbourhoodMap))
              multiCellIdMap(GridCellId(x, y)) = currentId + newCells.length
            }
          }
        }

        if (!cellDivided) {
          continuousEnvCell.obstacles = mergeToLocalObstacles(obstaclesGroups, x, y)
          continuousEnvCell.neighbourhood = updateNeighbourhoodWithObstacles(continuousEnvCell)

          worldBuilder.updateNeighbourhood(gridMultiCellId, continuousEnvCell.neighbourhood)

          finalCellQueue.enqueue(gridMultiCellId)
        }

      } else {
        finalCellQueue.enqueue(gridMultiCellId)
      }

      worldBuilder(gridMultiCellId) = CellState(continuousEnvCell)
    }

    var runners: List[RunnerDefinition] = List(
      //RunnerDefinition(535, 615, 10, 23),
     // RunnerDefinition(535, 655, 10, 23),
    );

    var base : Vec2  = new Vec2(270, 2150);
    var step : Vec2  = new Vec2(100,100);
    var rad = scala.util.Random.nextFloat() * 0 + 26;
    var speed = scala.util.Random.nextFloat() * 0 + 120;

    for(x <- 0 until  4)
      for(y <- 0 until 4)
      {
        var pos = base + step * new Vec2(x.doubleValue, y.doubleValue);
        runners = runners.appended(new RunnerDefinition(pos.x, pos.y, rad, speed, "D", Color.red));
      }

    var base2: Vec2 = new Vec2(2120, 250);

    for (x <- 0 until 4)
      for (y <- 0 until 4) {
        var pos = base2 + step * new Vec2(x.doubleValue, y.doubleValue);
        runners = runners.appended(new RunnerDefinition(pos.x, pos.y, rad, speed, "R", Color.green));
      }

    var base3: Vec2 = Vec2(4770, 2450);

    for (x <- 0 until 2)
      for (y <- 0 until 2)
      {
        var pos = base3 + step * new Vec2(-x.doubleValue, y.doubleValue);
        //runners = runners.appended(new RunnerDefinition(pos.x, pos.y, rad, speed, "U", Color.blue));
      }

    var base4: Vec2 = new Vec2(2450, 4750);

    for (x <- 0 until 2)
      for (y <- 0 until 2) {
        var pos = base4 + step * new Vec2(x.doubleValue, -y.doubleValue);
        //runners = runners.appended(new RunnerDefinition(pos.x, pos.y, rad, speed, "L", Color.magenta));
      }


//    for(x <- 0 to 10)
//      runners = runners.appended(new RunnerDefinition(base.x - x * 100, base.y, rad, speed, "A"));
//    for(x <- 0 to 10)
//      runners = runners.appended(new RunnerDefinition(base.x-x*100, base.y+1100, rad, speed, "B"));
//
//    for(x <- 0 to 10)
//      runners = runners.appended(new RunnerDefinition(base.x+500, base.y+x*100, rad, speed, "D"));
//    for(x <- 0 to 10)
    //runners = runners.appended(new RunnerDefinition(base.x-200, base.y+900  , rad, speed, "D"));


    while (finalCellQueue.nonEmpty) {
      val gridMultiCellId = finalCellQueue.dequeue()
      val continuousEnvCell: ContinuousEnvCell = worldBuilder(gridMultiCellId).state.contents.asInstanceOf[ContinuousEnvCell]
      var cellSize = config.cellSize
      var cellBase = continuousEnvCell.BaseCoordinates();
      runners.foreach(runner => {
          var runPos = new Vec2(runner.x, runner.y);
          var runInCellPos = Vec2((runPos - cellBase).x,cellSize-(runPos - cellBase).y);
          if(runInCellPos.x < cellSize && runInCellPos.y < cellSize && runInCellPos.x >= 0 && runInCellPos.y >= 0)
          {
            val runnerObj: Runner = Runner.createNew(runInCellPos, runner.radius, runner.maxSpeed, runner.color, runner.tag)
            continuousEnvCell.runners = continuousEnvCell.runners.appended(runnerObj)
            continuousEnvCell.coordinates += (runnerObj.id -> (runnerObj.positionInCell.x, runnerObj.positionInCell.y, runnerObj.radius, runnerObj.color))
          }
      });

      val boundaryObstacles = getBoundaryObstacles(continuousEnvCell)
      val allObstacles = boundaryObstacles ++ continuousEnvCell.obstacles

      if (boundaryObstacles.nonEmpty) {
        var obstaclesGroups: Array[Array[Obstacle]] = Array()

        for (obstacle <- allObstacles) {
          var obstaclesToMerge = Array(obstacle)
          var newObstaclesGroups: Array[Array[Obstacle]] = Array()
          for (existingObstacleGroup <- obstaclesGroups) {
            if (overlapsWithAny(obstacle, existingObstacleGroup)) {
              obstaclesToMerge = obstaclesToMerge ++ existingObstacleGroup
            } else {
              newObstaclesGroups = newObstaclesGroups :+ existingObstacleGroup
            }
          }
          newObstaclesGroups = newObstaclesGroups :+ obstaclesToMerge
          obstaclesGroups = newObstaclesGroups
        }

        val obstaclesInCell = mergeToObstacles(obstaclesGroups)
        var x = continuousEnvCell.gridMultiCellId.x
        var y = continuousEnvCell.gridMultiCellId.y
        if (x == 1 && y == 1) {
        }
        continuousEnvCell.obstacles = obstaclesInCell
//        continuousEnvCell.obstacles.foreach(obs => {
//          if(obs.xsOrig.size > 20)
//          println(obs.xsOrig.size)
//        })
      }

      val obstacles: Array[Obstacle] = continuousEnvCell.obstacles
        .filter(obstacle => obstacle.points > 0)
      if (obstacles.nonEmpty) {
        continuousEnvCell.graph = mapObstaclesToPathGraphVertices(obstacles, continuousEnvCell.cellOutline)
      }
      continuousEnvCell.cardinalSegments = mapSegmentsToProperCoords(continuousEnvCell.neighbourhood.cardinalNeighbourhood, config.cellSize)

      worldBuilder(gridMultiCellId) = CellState(continuousEnvCell)
    }

    worldBuilder
  }

  private def GetGlobalBorderObstacles()(implicit config: ContinuousEnvConfig): List[Obstacle] =
  {
    var size = 100;
    var max : Vec2Int = Vec2Int(config.worldWidth * config.cellSize, config.worldHeight * config.cellSize)
    return List[Obstacle](
      Obstacle(Array(0,max.x, max.x, 0), Array(-size, -size, 0, 0), 4),
      Obstacle(Array(max.x, max.x + size, max.x + size, max.x), Array(0, 0, max.y, max.y), 4),
      Obstacle(Array(0, max.x, max.x, 0), Array(max.y, max.y, max.y + size,max.y + size), 4),
      Obstacle(Array(-size, 0, 0, -size), Array(0, 0, max.y, max.y), 4)
    );
  }

  private def FixGlobalObstacles(obstacles: List[Obstacle])(implicit config: ContinuousEnvConfig): List[Obstacle] = {
    val mapSize = config.worldWidth * config.cellSize
    obstacles.map(obs => Obstacle(obs.xs, obs.ys.map(y => mapSize - y), obs.points));
  }

  private def mapSegmentsToProperCoords(cardinalNeighbourhood: Map[GridDirection, Boundary], cellSize: Int): Map[Line, GridMultiCellId] = {
    var result: Map[Line, GridMultiCellId] = Map.empty
    cardinalNeighbourhood
      .foreach(directionState => directionState._2.boundaries
        .foreach(segmentEntry => result += (segmentToLine(segmentEntry._1, directionState._1, cellSize) -> segmentEntry._2)))
    result
  }

  private def segmentToLine(segment: Segment, gridDirection: GridDirection, cellSize: Int): Line = {
    gridDirection match {
      case Left => Line(Vec2(0, segment.a), Vec2(0, segment.b))
      case Right => Line(Vec2(cellSize, segment.a), Vec2(cellSize, segment.b))
      case Top => Line(Vec2(segment.a, cellSize), Vec2(segment.b, cellSize))
      case Bottom => Line(Vec2(segment.a, 0), Vec2(segment.b, 0))
      case _ => throw new RuntimeException("Direction is not cardinal")
    }
  }

  /*private def segmentToLine(segment: Segment, gridDirection: GridDirection, cellSize: Int): Line = {
    gridDirection match {
      case Left => Line(Vec2(segment.a, cellSize), Vec2(segment.b, cellSize))
      case Right => Line(Vec2(segment.a, 0), Vec2(segment.b, 0))
      case Top => Line(Vec2(0, cellSize - segment.b), Vec2(0, cellSize - segment.a))
      case Bottom => Line(Vec2(cellSize, cellSize - segment.b), Vec2(cellSize, cellSize - segment.a))
      case _ => throw new RuntimeException("Direction is not cardinal")
    }
  }*/

  private def mapObstaclesToPathGraphVertices(obstacles: Array[Obstacle], cellOutline: CellOutline): Map[Vec2, Set[Vec2]] = {
    val result: MutableMap[Vec2, Set[Vec2]] = MutableMap.empty
    val graphSegments: Set[Line] = getVoronoiSegments(prepareVoronoiInput(obstacles, cellOutline, 10), cellOutline)
      .filter(line => !isLineInsideObstacleOrOutline(obstacles, line, cellOutline))
    var vertices: Set[Vec2] = Set.empty
    graphSegments.foreach(line => vertices = vertices ++ Set(line.start, line.end))
    vertices.foreach(v => result(v) = getVerticeNeighbours(v, graphSegments))
    result.toMap
  }

  private def getVerticeNeighbours(v: Vec2, lines: Set[Line]): Set[Vec2] = {
    var result: Set[Vec2] = Set.empty
    lines.foreach(line => {
      if (line.start.equals(v)) {
        result = result ++ Set(line.end)
      }
      if (line.end.equals(v)) {
        result = result ++ Set(line.start)
      }
    })
    result
  }

  private def prepareVoronoiInput(obstacles: Array[Obstacle], cellOutline: CellOutline, outlineSteps: Int): List[(Int, Int)] = {
    var voronoiInput: Array[(Int, Int)] = obstacles.map(obstacle => obstacle.xs zip obstacle.ys)
      .flatMap(vertices => getObstacleVoronoiInput(vertices))
    for (i <- cellOutline.x - 1 until cellOutline.x + cellOutline.width + 2 by max(cellOutline.width / outlineSteps, 1)) {
      voronoiInput = voronoiInput :+ (i, cellOutline.y - 1)
      voronoiInput = voronoiInput :+ (i, cellOutline.y + cellOutline.height + 1)
    }
    for (i <- cellOutline.y - 1 until cellOutline.y + cellOutline.height + 2 by max(cellOutline.height / outlineSteps, 1)) {
      voronoiInput = voronoiInput :+ (cellOutline.x - 1, i)
      voronoiInput = voronoiInput :+ (cellOutline.x + cellOutline.width + 1, i)
    }
    voronoiInput.toList
  }

  private def getObstacleVoronoiInput(vertices: Array[(Int, Int)]): Array[(Int, Int)] = {
    var result: Array[(Int, Int)] = Array()
    for (i <- 0 until vertices.length - 1) {
      val center = ((vertices(i)._1 + vertices(i + 1)._1) / 2, (vertices(i)._2 + vertices(i + 1)._2) / 2)
      result = result :+ vertices(i)
      result = result :+ vertices(i + 1)
      result = result :+ center
      result = result :+ ((center._1 + vertices(i + 1)._1) / 2, (center._2 + vertices(i + 1)._2) / 2)
      result = result :+ ((vertices(i)._1 + center._1) / 2, (vertices(i)._2 + center._2) / 2)
    }
    result = result :+ ((vertices(vertices.length - 1)._1 + vertices(0)._1) / 2, (vertices(vertices.length - 1)._2 + vertices(0)._2) / 2)
    result
  }

  private def isLineInsideObstacleOrOutline(obstacles: Array[Obstacle], line: Line, cellOutline: CellOutline): Boolean = {
    if (isLineOnOutline(line, cellOutline)) return true
    obstacles.map(obstacle => new Polygon(obstacle.xs, obstacle.ys, obstacle.xs.length))
      .exists(polygon => polygon.contains(line.start.x, line.start.y) || polygon.contains(line.end.x, line.end.y))
  }

  private def isLineOnOutline(line: Line, cellOutline: CellOutline): Boolean = {
    line.start.x == cellOutline.x - 1 || line.start.x == cellOutline.x + cellOutline.width + 1 ||
      line.end.x == cellOutline.x - 1 || line.end.x == cellOutline.x + cellOutline.width + 1 ||
      line.start.y == cellOutline.y - 1 || line.start.y == cellOutline.y + cellOutline.height + 1 ||
      line.end.y == cellOutline.y - 1 || line.end.y == cellOutline.y + cellOutline.height + 1
  }

  private def getVoronoiSegments(vertices: List[(Int, Int)], cellOutline: CellOutline): Set[Line] = {
    val coordinates = vertices map { p => new Coordinate(p._1, p._2) }
    val voronoi = new VoronoiDiagramBuilder()
    voronoi.setSites(coordinates.asJava)
    val envelope = new Envelope(new Coordinate(cellOutline.x - 1, cellOutline.y - 1),
      new Coordinate(cellOutline.x + cellOutline.width + 1, cellOutline.y + cellOutline.height + 1))
    voronoi.setClipEnvelope(envelope)
    val geometry = voronoi.getDiagram(new GeometryFactory()).asInstanceOf[GeometryCollection]
    val polygons = for (i <- 0 until geometry.getNumGeometries)
      yield geometry.getGeometryN(i).asInstanceOf[org.locationtech.jts.geom.Polygon]
    polygons.map(polygon => polygon.getCoordinates)
      .map(polygon => polygon.map(vertice => Vec2(vertice.x, vertice.y)))
      .flatMap(polygon => mapCoordsToSegments(polygon))
      .toSet
  }

  private def mapCoordsToSegments(coords: Array[Vec2]): Array[Line] = {
    var result: Array[Line] = Array()
    for (i <- 0 until coords.length - 1) {
      result = result :+ Line(coords(i), coords(i + 1))
    }
    result
  }

  private def getBoundaryObstacles(cell: ContinuousEnvCell)
                                  (implicit config: ContinuousEnvConfig): Array[Obstacle] = {
    cell.neighbourhood.cardinalNeighbourhood
      .map { case (direction, boundary) => getBoundaryObstaclesInDirection(direction, boundary, cell) }
      .flatten
      .toArray
  }

  private def getBoundaryObstaclesInDirection(direction: GridDirection, boundary: Boundary, cell: ContinuousEnvCell)
                                             (implicit config: ContinuousEnvConfig): Array[Obstacle] = {
    getNonBoundarySegments(boundary, direction, cell)
      .map(segment => getBoundaryObstacleInDirection(direction, segment, cell))
  }

  private def getNonBoundarySegments(boundary: Boundary, direction: GridDirection, cell: ContinuousEnvCell)
                                    (implicit config: ContinuousEnvConfig): Array[Segment] = {
    var nonBoundarySegments: Array[Segment] = if (direction == Top || direction == Bottom) {
      Array(Segment(cell.cellOutline.x, cell.cellOutline.x + cell.cellOutline.width))
    } else {
      Array(Segment(cell.cellOutline.y, cell.cellOutline.y + cell.cellOutline.height))
    }

    for (boundarySegment <- boundary.boundaries.keys) {
      var newNonBoundarySegments: Array[Segment] = Array()
      for (nonBoundarySegment <- nonBoundarySegments) {
        if (nonBoundarySegment.a < boundarySegment.a && nonBoundarySegment.b > boundarySegment.b) {
          val leftSegment = Segment(nonBoundarySegment.a, boundarySegment.a)
          val rightSegment = Segment(boundarySegment.b, nonBoundarySegment.b)
          newNonBoundarySegments = newNonBoundarySegments :+ leftSegment :+ rightSegment
        } else if (nonBoundarySegment.a < boundarySegment.a && nonBoundarySegment.b <= boundarySegment.b) {
          val leftSegment = Segment(nonBoundarySegment.a, boundarySegment.a)
          newNonBoundarySegments = newNonBoundarySegments :+ leftSegment
        } else if (nonBoundarySegment.a >= boundarySegment.a && nonBoundarySegment.b > boundarySegment.b) {
          val rightSegment = Segment(boundarySegment.b, nonBoundarySegment.b)
          newNonBoundarySegments = newNonBoundarySegments :+ rightSegment
        } else if (nonBoundarySegment.a >= boundarySegment.a && nonBoundarySegment.b <= boundarySegment.b) {
          // nothing
        }
      }
      nonBoundarySegments = newNonBoundarySegments
    }

    nonBoundarySegments
  }

  private def getBoundaryObstacleInDirection(direction: GridDirection, segment: Segment, cell: ContinuousEnvCell)
                                            (implicit config: ContinuousEnvConfig): Obstacle = {
    direction match {
      case Top =>
        val xs = Array(segment.a, segment.a, segment.b, segment.b)
        val ys = Array(0, 1, 1, 0).map(y => y + cell.cellOutline.y + cell.cellOutline.height)

        Obstacle(xs, ys, 4)
      case Right =>
        val xs = Array(0, 1, 1, 0).map(x => x + cell.cellOutline.x + cell.cellOutline.width)
        val ys = Array(segment.b, segment.b, segment.a, segment.a)

        Obstacle(xs, ys, 4)
      case Bottom =>
        val xs = Array(segment.a, segment.b, segment.b, segment.a)
        val ys = Array(0, 0, -1, -1).map(y => y + cell.cellOutline.y)

        Obstacle(xs, ys, 4)
      case Left =>
        val xs = Array(0, -1, -1, 0).map(x => x + cell.cellOutline.x)
        val ys = Array(segment.a, segment.a, segment.b, segment.b)

        Obstacle(xs, ys, 4)
      case _ => null
    }
  }

  private def bufferObstacles(obstacles: List[Obstacle])
                             (implicit continuousEnvConfig: ContinuousEnvConfig): List[Obstacle] = {
    obstacles
      .map(obstacle => bufferObstacle(obstacle))
  }

  private def bufferObstacle(obstacle: Obstacle)
                            (implicit config: ContinuousEnvConfig): Obstacle = {
    var jtsCoordinates: Array[Coordinate] = Range(0, obstacle.points)
      .map(i => new Coordinate(obstacle.xs(i), obstacle.ys(i)))
      .toArray
    jtsCoordinates = jtsCoordinates :+ new Coordinate(obstacle.xs(0), obstacle.ys(0))

    val geometryFactory = new GeometryFactory()
    val shell = geometryFactory.createPolygon(jtsCoordinates)
    val bufferedPolygon = shell.buffer(config.maxAgentRadius, BufferParameters.CAP_FLAT)

    var newXs: Array[Int] = Array()
    var newYs: Array[Int] = Array()

    bufferedPolygon
      .getCoordinates
      .take(bufferedPolygon.getCoordinates.length - 1)
      .foreach(coordinate => {
        newXs = newXs :+ coordinate.x.intValue
        newYs = newYs :+ coordinate.y.intValue
      })

    Obstacle(newXs, newYs, newXs.length)
  }

  private def getOverlappingObstacles(continuousEnvCell: ContinuousEnvCell, obstacles: List[Obstacle], x: Int, y: Int)
                                     (implicit config: ContinuousEnvConfig): List[Obstacle] = {
    val cellOutline = continuousEnvCell.cellOutline

    val overlappingObstaclesAreas = getOverlappingGlobalObstaclesAndAreas(continuousEnvCell, obstacles, x, y)
      .map(overlappingObstacleArea => toObstacle(overlappingObstacleArea._1))
      .map(obstacle => fixArtifactsInObstacle(obstacle, cellOutline, x, y))

    overlappingObstaclesAreas
  }

  private def getOverlappingGlobalObstacles(continuousEnvCell: ContinuousEnvCell, obstacles: List[Obstacle], x: Int, y: Int)
                                                   (implicit config: ContinuousEnvConfig): List[Obstacle] = {
    getOverlappingGlobalObstaclesAndAreas(continuousEnvCell, obstacles, x, y).map(obstacleArea => obstacleArea._2);
  }

  private def getOverlappingGlobalObstaclesAndAreas(continuousEnvCell: ContinuousEnvCell, obstacles: List[Obstacle], x: Int, y: Int)
                                     (implicit config: ContinuousEnvConfig): List[(Area, Obstacle)] = {
    val cellOutline = continuousEnvCell.cellOutline
    val xScale = y
    val yScale = config.worldWidth - x - 1

    val cellOutlineArea = new Area(new Rectangle(
      xScale * config.cellSize + cellOutline.x.intValue, yScale * config.cellSize + cellOutline.y.intValue,
      cellOutline.width.intValue, cellOutline.height.intValue))
    val obstaclesAreas = obstacles
      .map(obstacle => (new Area(new Polygon(obstacle.xs, obstacle.ys, obstacle.points)), obstacle))
    obstaclesAreas
      .foreach(obstacleArea => (obstacleArea._1.intersect(cellOutlineArea), obstacleArea._2))
    val overlappingObstaclesAreas = obstaclesAreas
      .filter(obstacleArea => !obstacleArea._1.isEmpty)
    overlappingObstaclesAreas
  }

  private def fixArtifactsInObstacle(obstacle: Obstacle, cellOutline: CellOutline, x: Int, y: Int)
                                    (implicit config: ContinuousEnvConfig): Obstacle = {
    val xScale = y
    val yScale = config.worldWidth - x - 1
    val xBoundaries: Array[Int] = Array(
      xScale * config.cellSize + cellOutline.x.intValue,
      xScale * config.cellSize + cellOutline.x.intValue + cellOutline.width.intValue,
    )
    val yBoundaries: Array[Int] = Array(
      yScale * config.cellSize + cellOutline.y.intValue,
      yScale * config.cellSize + cellOutline.y.intValue + cellOutline.height.intValue
    )

    var newXs: Array[Int] = Array()
    var newYs: Array[Int] = Array()

    for (i <- 0 until obstacle.points) {
      var newX = obstacle.xs(i)
      var newY = obstacle.ys(i)

      if (math.abs(newX - xBoundaries(0)) == 1) {
        newX = xBoundaries(0)
      } else if (math.abs(newX - xBoundaries(1)) == 1) {
        newX = xBoundaries(1)
      }

      if (math.abs(newY - yBoundaries(0)) == 1) {
        newY = yBoundaries(0)
      } else if (math.abs(newY - yBoundaries(1)) == 1) {
        newY = yBoundaries(1)
      }

      newXs = newXs :+ newX
      newYs = newYs :+ newY
    }

    var uniquePoints: Array[(Int, Int)] = Array()
    for (i <- 0 until obstacle.points) {
      uniquePoints = uniquePoints :+ (newXs(i), newYs(i))
    }
    uniquePoints = uniquePoints.distinct

    newXs = Array()
    newYs = Array()

    uniquePoints.foreach(point => {
      newXs = newXs :+ point._1
      newYs = newYs :+ point._2
    }
    )

    Obstacle(newXs, newYs, newXs.length)
  }

  private def toObstacle(overlappingObstacleArea: Area): Obstacle = {
    val pathIterator = overlappingObstacleArea.getPathIterator(null)
    val coords = Array(0d, 0d, 0d, 0d, 0d, 0d)
    var xs: Array[Int] = Array()
    var ys: Array[Int] = Array()
    while (!pathIterator.isDone) {
      pathIterator.currentSegment(coords)
      xs = xs :+ coords(0).intValue
      ys = ys :+ coords(1).intValue
      pathIterator.next()
    }

    xs = xs.dropRight(1)
    ys = ys.dropRight(1)

    Obstacle(xs, ys, xs.length)
  }


  private def overlapsWithAny(obstacle: Obstacle, obstaclesGroup: Array[Obstacle]): Boolean = {
    obstaclesGroup.exists(existingObstacle => overlaps(obstacle, existingObstacle))
  }

  private def overlaps(obstacle: Obstacle, existingObstacle: Obstacle): Boolean = {
    anyPointLiesIn(obstacle, existingObstacle) ||
      anyPointLiesIn(existingObstacle, obstacle) ||
      anyPointLiesOn(obstacle, existingObstacle) ||
      anyPointLiesOn(existingObstacle, obstacle)
  }

  private def anyPointLiesIn(first: Obstacle, second: Obstacle): Boolean = {
    var result = false
    val poly = new Polygon(first.xs, first.ys, first.points)
    for (i <- 0 until second.points) {
      if (poly.contains(second.xs(i), second.ys(i))) {
        result = true
      }
    }

    result
  }

  private def anyPointLiesOn(first: Obstacle, second: Obstacle): Boolean = {
    var result = false
    for (i <- 0 until first.points) {
      val cX = first.xs(i)
      val cY = first.ys(i)
      for (j <- 0 until second.points) {
        val aX = second.xs(j)
        val aY = second.ys(j)
        val bX = second.xs((j + 1) % second.points)
        val bY = second.ys((j + 1) % second.points)

        if (squaredDistance(aX, aY, cX, cY) + squaredDistance(cX, cY, bX, bY) == squaredDistance(aX, aY, bX, bY)) {
          result = true
        }
      }
    }

    result
  }

  private def squaredDistance(x1: Int, y1: Int, x2: Int, y2: Int): Int = {
    (x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2)
  }

  private def isObstaclesGroupDividingCell(cellOutline: CellOutline, obstaclesGroup: Array[Obstacle], cellX: Int, cellY: Int)
                                          (implicit config: ContinuousEnvConfig): Boolean = {
    var obstacle = mergeToObstacle(obstaclesGroup)
    obstacle = addDummyPointsBetweenPointsLyingOnEdges(cellOutline, obstacle, cellX, cellY)
    val flags = pointsOnCellOutline(cellOutline, obstacle, cellX, cellY)
    val flips = countFlips(flags)

    flips > 2
  }


  private def mergeToObstacle(obstaclesGroup: Array[Obstacle]): Obstacle = {
    val mergedObstacleArea = new Area()
    for (obstacle <- obstaclesGroup) {
      val obstaclePoly = new Polygon(obstacle.xs, obstacle.ys, obstacle.points)
      mergedObstacleArea.add(new Area(obstaclePoly))
    }
    toObstacle(mergedObstacleArea)
  }

  private def addDummyPointsBetweenPointsLyingOnEdges(cellOutline: CellOutline, obstacle: Obstacle, cellX: Int, cellY: Int)
                                                     (implicit config: ContinuousEnvConfig): Obstacle = {
    var newXs: Array[Int] = Array()
    var newYs: Array[Int] = Array()

    for (i <- 0 until obstacle.points) {
      val currentX = obstacle.xs(i)
      val currentY = obstacle.ys(i)
      val nextX = obstacle.xs((i + 1) % obstacle.points)
      val nextY = obstacle.ys((i + 1) % obstacle.points)
      if (!isLyingInsideCellOutline(cellOutline, currentX, currentY, cellX, cellY)
        && !isLyingInsideCellOutline(cellOutline, nextX, nextY, cellX, cellY)) {
        val dummyX = (currentX + nextX) / 2
        val dummyY = (currentY + nextY) / 2

        newXs = newXs :+ currentX :+ dummyX
        newYs = newYs :+ currentY :+ dummyY
      } else {
        newXs = newXs :+ currentX
        newYs = newYs :+ currentY
      }
    }

    Obstacle(newXs, newYs, newXs.length)
  }

  private def isLyingInsideCellOutline(cellOutline: CellOutline, x: Int, y: Int, cellX: Int, cellY: Int)
                                      (implicit config: ContinuousEnvConfig): Boolean = {
    val xScale = cellY
    val yScale = config.worldWidth - cellX - 1

    val localX = x - xScale * config.cellSize
    val localY = y - yScale * config.cellSize
    localX > cellOutline.x && localX < cellOutline.x + cellOutline.width && localY > cellOutline.y && localY < cellOutline.y + cellOutline.height
  }

  private def pointsOnCellOutline(cellOutline: CellOutline, obstacle: Obstacle, cellX: Int, cellY: Int)
                                 (implicit config: ContinuousEnvConfig): Array[Boolean] = {
    var flags: Array[Boolean] = Array()
    for (i <- 0 until obstacle.points) {
      val currentX = obstacle.xs(i)
      val currentY = obstacle.ys(i)
      flags = flags :+ !isLyingInsideCellOutline(cellOutline, currentX, currentY, cellX, cellY)
    }

    flags
  }

  private def countFlips(flags: Array[Boolean]): Int = {
    var flips = 0
    for (i <- flags.indices) {
      if (flags(i) != flags((i + 1) % flags.length)) {
        flips = flips + 1
      }
    }

    flips
  }

  private def divideCells(cell: ContinuousEnvCell, obstaclesGroup: Array[Obstacle], cellX: Int, cellY: Int)
                         (implicit config: ContinuousEnvConfig): Array[ContinuousEnvCell] = {
    var obstacle = mergeToObstacle(obstaclesGroup)
    obstacle = addDummyPointsBetweenPointsLyingOnEdges(cell.cellOutline, obstacle, cellX, cellY)
    val flags = pointsOnCellOutline(cell.cellOutline, obstacle, cellX, cellY)

    val newCellBoundaries: Array[Array[(Int, Int)]] = extractNewCellBoundaries(cell.cellOutline, obstacle, flags, cellX, cellY)

    val newCells = newCellBoundaries
      .map(cellBoundary => createNewCell(cell, cellBoundary))

    newCells
  }

  private def extractNewCellBoundaries(cellOutline: CellOutline, obstacle: Obstacle, flags: Array[Boolean], cellX: Int, cellY: Int)
                                      (implicit config: ContinuousEnvConfig): Array[Array[(Int, Int)]] = {
    val localObstaclePoints: Array[(Int, Int)] = toLocalObstaclePoints(obstacle, cellX, cellY)
    var newCellBoundaries: Array[Array[(Int, Int)]] = Array()
    val newCellsNum = countFlips(flags) / 2

    var start = 0
    for (_ <- 0 until newCellsNum) {
      val nextTrueBeforeFalse = findNextTrueBeforeFalse(flags, start)
      val nextTrue = findNextTrue(flags, nextTrueBeforeFalse + 1)

      var cellSplit: Array[(Int, Int)] = Array()
      for (j <- nextTrueBeforeFalse to nextTrue) {
        cellSplit = cellSplit :+ localObstaclePoints(j % localObstaclePoints.length)
      }

      cellSplit = addCellOutlineVerticesToSplit(cellOutline, cellSplit)

      newCellBoundaries = newCellBoundaries :+ cellSplit
      start = nextTrue + 1
    }

    newCellBoundaries
  }

  private def toLocalObstaclePoints(obstacle: Obstacle, cellX: Int, cellY: Int)
                                   (implicit config: ContinuousEnvConfig): Array[(Int, Int)] = {
    var localObstaclePoints: Array[(Int, Int)] = Array()
    for (i <- 0 until obstacle.points) {
      val xScale = cellY
      val yScale = config.worldWidth - cellX - 1

      val localX = obstacle.xs(i) - xScale * config.cellSize
      val localY = obstacle.ys(i) - yScale * config.cellSize

      localObstaclePoints = localObstaclePoints :+ (localX, localY)
    }

    localObstaclePoints
  }

  private def toLocalObstacle(obstacle: Obstacle, cellX: Int, cellY: Int)
                             (implicit config: ContinuousEnvConfig): Obstacle = {
    var localObstaclePointsX: Array[Int] = Array()
    var localObstaclePointsY: Array[Int] = Array()
    for (i <- 0 until obstacle.points) {
      val xScale = cellY
      val yScale = config.worldWidth - cellX - 1

      val localX = obstacle.xs(i) - xScale * config.cellSize
      val localY = obstacle.ys(i) - yScale * config.cellSize

      localObstaclePointsX = localObstaclePointsX :+ localX
      localObstaclePointsY = localObstaclePointsY :+ localY
    }

    Obstacle(localObstaclePointsX, localObstaclePointsY, localObstaclePointsX.length)
  }

  private def findNextTrueBeforeFalse(flags: Array[Boolean], start: Int): Int = {
    var result = -1
    var found = false
    for (i <- 0 to flags.length if !found) {
      if (flags((start + i) % flags.length) && !flags((start + i + 1) % flags.length)) {
        result = start + i
        found = true
      }
    }

    result
  }

  private def findNextTrue(flags: Array[Boolean], start: Int): Int = {
    var result = -1
    var found = false
    for (i <- 0 to flags.length if !found) {
      if (flags((start + i) % flags.length)) {
        result = start + i
        found = true
      }
    }

    result
  }

  private def addCellOutlineVerticesToSplit(cellOutline: CellOutline, cellSplit: Array[(Int, Int)]): Array[(Int, Int)] = {
    val firstBoundaryNum = getBoundaryNumForPoint(cellOutline, cellSplit(0))
    var lastBoundaryNum = getBoundaryNumForPoint(cellOutline, cellSplit(cellSplit.length - 1))

    val cellOutlineVertices: Array[(Int, Int)] = Array((cellOutline.x, cellOutline.y + cellOutline.height),
      (cellOutline.x + cellOutline.width, cellOutline.y + cellOutline.height),
      (cellOutline.x + cellOutline.width, cellOutline.y),
      (cellOutline.x, cellOutline.y))

    var verticesToAdd: Array[(Int, Int)] = Array()
    if (lastBoundaryNum < firstBoundaryNum) {
      lastBoundaryNum = lastBoundaryNum + 4
    }

    try {
      for (i <- firstBoundaryNum until lastBoundaryNum) {
        if (cellOutlineVertices(i % 4) != cellSplit(0)) {
          verticesToAdd = verticesToAdd :+ cellOutlineVertices(i % 4)
        }
      }
    } catch {
      case _: Exception =>
        println("oops")
    }

    cellSplit.reverse :++ verticesToAdd
  }

  private def getBoundaryNumForPoint(cellOutline: CellOutline, point: (Int, Int)): Int = {
    val x = point._1
    val y = point._2

    var boundaryNum = -1
    if (x == cellOutline.x && y > cellOutline.y && y <= cellOutline.y + cellOutline.height) {
      boundaryNum = 0
    } else if (x > cellOutline.x && x <= cellOutline.x + cellOutline.width && y == cellOutline.y + cellOutline.height) {
      boundaryNum = 1
    } else if (x == cellOutline.x + cellOutline.width && y >= cellOutline.y && y < cellOutline.y + cellOutline.height) {
      boundaryNum = 2
    } else if (x <= cellOutline.x + cellOutline.width && x > cellOutline.x && y == cellOutline.y) {
      boundaryNum = 3
    }

    boundaryNum
  }

  private def createNewCell(existingCell: ContinuousEnvCell, newCellBoundary: Array[(Int, Int)])
                           (implicit config: ContinuousEnvConfig): ContinuousEnvCell = {
    val existingNeighbourhood = existingCell.neighbourhood
    var newCellNeighbourhood = Neighbourhood.empty()
    val cardinalNeighbourhood: MutableMap[GridDirection, Boundary] = MutableMap.from(newCellNeighbourhood.cardinalNeighbourhood)
    val diagonalNeighbourhood: MutableMap[GridDirection, GridMultiCellId] = MutableMap.from(newCellNeighbourhood.diagonalNeighbourhood)

    for (i <- newCellBoundary.indices) {
      val start = newCellBoundary(i)
      val end = newCellBoundary((i + 1) % newCellBoundary.length)
      if (isVertical(start, end)) { // can be left or right
        var segment = Segment(start._2, end._2)
        if (isLeft(existingCell, start)) {
          cardinalNeighbourhood(Left) = getNewCardinalBoundary(existingNeighbourhood, Left, segment)
        } else if (isRight(existingCell, start)) {
          segment = Segment(segment.b, segment.a) // need to mirror in case it's right boundary
          cardinalNeighbourhood(Right) = getNewCardinalBoundary(existingNeighbourhood, Right, segment)
        }
      } else if (isHorizontal(start, end)) { // can be top or bottom
        var segment = Segment(start._1, end._1)
        if (isTop(existingCell, start)) {
          cardinalNeighbourhood(Top) = getNewCardinalBoundary(existingNeighbourhood, Top, segment)
        } else if (isBottom(existingCell, start)) {
          segment = Segment(segment.b, segment.a) // need to mirror in case it's bottom boundary
          cardinalNeighbourhood(Bottom) = getNewCardinalBoundary(existingNeighbourhood, Bottom, segment)
        }
      }
    }

    for (i <- newCellBoundary.indices) {
      val point = newCellBoundary(i)
      if (isTopLeft(existingCell, point)) {
        diagonalNeighbourhood(TopLeft) = existingNeighbourhood.diagonalNeighbourhood(TopLeft)
      } else if (isTopRight(existingCell, point)) {
        diagonalNeighbourhood(TopRight) = existingNeighbourhood.diagonalNeighbourhood(TopRight)
      } else if (isBottomRight(existingCell, point)) {
        diagonalNeighbourhood(BottomRight) = existingNeighbourhood.diagonalNeighbourhood(BottomRight)
      } else if (isBottomLeft(existingCell, point)) {
        diagonalNeighbourhood(BottomLeft) = existingNeighbourhood.diagonalNeighbourhood(BottomLeft)
      }
    }

    newCellNeighbourhood = Neighbourhood(Map.from(cardinalNeighbourhood), Map.from(diagonalNeighbourhood))
    val newCell = ContinuousEnvCell(existingCell.initialSignal, existingCell.gridMultiCellId)
    newCell.cellOutline = getNewCellOutline(newCellBoundary)
    newCell.neighbourhood = newCellNeighbourhood

    newCell
  }

  private def isVertical(start: (Int, Int), end: (Int, Int)): Boolean = {
    start._1 == end._1
  }

  private def isHorizontal(start: (Int, Int), end: (Int, Int)): Boolean = {
    start._2 == end._2
  }

  private def isLeft(continuousEnvCell: ContinuousEnvCell, point: (Int, Int)): Boolean = {
    continuousEnvCell.cellOutline.x == point._1
  }

  private def isRight(continuousEnvCell: ContinuousEnvCell, point: (Int, Int)): Boolean = {
    continuousEnvCell.cellOutline.x + continuousEnvCell.cellOutline.width == point._1
  }

  private def isTop(continuousEnvCell: ContinuousEnvCell, point: (Int, Int)): Boolean = {
    continuousEnvCell.cellOutline.y + continuousEnvCell.cellOutline.height == point._2
  }

  private def isBottom(continuousEnvCell: ContinuousEnvCell, point: (Int, Int)): Boolean = {
    continuousEnvCell.cellOutline.y == point._2
  }

  private def isTopLeft(continuousEnvCell: ContinuousEnvCell, point: (Int, Int)): Boolean = {
    val cellOutline = continuousEnvCell.cellOutline
    cellOutline.x == point._1 && cellOutline.y + cellOutline.height == point._2
  }

  private def isTopRight(continuousEnvCell: ContinuousEnvCell, point: (Int, Int)): Boolean = {
    val cellOutline = continuousEnvCell.cellOutline
    cellOutline.x + cellOutline.width == point._1 && cellOutline.y + cellOutline.height == point._2
  }

  private def isBottomRight(continuousEnvCell: ContinuousEnvCell, point: (Int, Int)): Boolean = {
    val cellOutline = continuousEnvCell.cellOutline
    cellOutline.x + cellOutline.width == point._1 && cellOutline.y == point._2
  }

  private def isBottomLeft(continuousEnvCell: ContinuousEnvCell, point: (Int, Int)): Boolean = {
    val cellOutline = continuousEnvCell.cellOutline
    cellOutline.x == point._1 && cellOutline.y == point._2
  }

  private def getNewCellOutline(boundary: Array[(Int, Int)]): CellOutline = {
    val minX = boundary.map(point => point._1).min
    val maxX = boundary.map(point => point._1).max
    val minY = boundary.map(point => point._2).min
    val maxY = boundary.map(point => point._2).max

    CellOutline(minX, minY, maxX - minX, maxY - minY)
  }

  private def getNewCardinalBoundary(existingNeighbourhood: Neighbourhood, cardinalDirection: GridDirection, segment: Segment): Boundary = {
    val boundaryMap = existingNeighbourhood
      .cardinalNeighbourhood(cardinalDirection)
      .boundaries
      .map { case (boundarySegment, gridMultiCellId) => (getCommonSegment(segment, boundarySegment), gridMultiCellId) }
      .filter { case (commonSegment, _) => commonSegment != null }

    Boundary(boundaryMap)
  }

  private def getCommonSegment(segment: Segment, boundarySegment: Segment): Segment = {
    val a = max(segment.a, boundarySegment.a)
    val b = scala.math.min(segment.b, boundarySegment.b)

    if (a > b) {
      null
    } else {
      Segment(a, b)
    }
  }

  private def hasEmptyNeighbourhood(continuousEnvCell: ContinuousEnvCell): Boolean = {
    val emptyDiagonalNeighbourhood = continuousEnvCell
      .neighbourhood
      .diagonalNeighbourhood
      .forall { case (_, gridMultiCellId) => gridMultiCellId == null }
    val emptyCardinalNeighbourhood = continuousEnvCell
      .neighbourhood
      .cardinalNeighbourhood
      .forall { case (_, boundary) => boundary.boundaries == Map.empty }

    emptyDiagonalNeighbourhood && emptyCardinalNeighbourhood
  }

  private def hasSameNeighbourhood(cell: ContinuousEnvCell, other: ContinuousEnvCell): Boolean = {
    val hasSameDiagonalNeighbourhood = cell
      .neighbourhood
      .diagonalNeighbourhood
      .forall { case (direction, gridMultiCellId) => gridMultiCellId == other.neighbourhood.diagonalNeighbourhood(direction) }
    val hasSameCardinalNeighbourhood = cell
      .neighbourhood
      .cardinalNeighbourhood
      .forall { case (direction, boundary) => boundary.boundaries == other.neighbourhood.cardinalNeighbourhood(direction).boundaries }

    hasSameDiagonalNeighbourhood && hasSameCardinalNeighbourhood
  }

  private def mergeToLocalObstacles(obstaclesGroups: Array[Array[Obstacle]], cellX: Int, cellY: Int)
                                   (implicit config: ContinuousEnvConfig): Array[Obstacle] = {
    mergeToObstacles(obstaclesGroups)
      .map(obstacle => toLocalObstacle(obstacle, cellX, cellY))
  }

  private def mergeToObstacles(obstaclesGroups: Array[Array[Obstacle]]): Array[Obstacle] = {
    obstaclesGroups
      .map(group => mergeToObstacle(group))
  }

  private def updateNeighbourhoodWithObstacles(continuousEnvCell: ContinuousEnvCell): Neighbourhood = {
    val cardinalNeighbourhood = MutableMap.from(continuousEnvCell.neighbourhood.cardinalNeighbourhood)
    val diagonalNeighbourhood = MutableMap.from(continuousEnvCell.neighbourhood.diagonalNeighbourhood)

    for (obstacle <- continuousEnvCell.obstacles) {
      for (i <- 0 until obstacle.points) {
        val start = (obstacle.xs(i), obstacle.ys(i))
        val end = (obstacle.xs((i + 1) % obstacle.points), obstacle.ys((i + 1) % obstacle.points))
        if (isVertical(start, end)) { // can be left or right
          var segment = Segment(start._2, end._2)
          if (isLeft(continuousEnvCell, start)) {
            cardinalNeighbourhood(Left) = updateBoundaryWithObstacleSegment(cardinalNeighbourhood(Left), segment)
          } else if (isRight(continuousEnvCell, start)) {
            segment = Segment(segment.b, segment.a) // need to mirror in case it's right boundary
            cardinalNeighbourhood(Right) = updateBoundaryWithObstacleSegment(cardinalNeighbourhood(Right), segment)
          }
        } else if (isHorizontal(start, end)) { // can be top or bottom
          var segment = Segment(start._1, end._1)
          if (isTop(continuousEnvCell, start)) {
            cardinalNeighbourhood(Top) = updateBoundaryWithObstacleSegment(cardinalNeighbourhood(Top), segment)
          } else if (isBottom(continuousEnvCell, start)) {
            segment = Segment(segment.b, segment.a) // need to mirror in case it's bottom boundary
            cardinalNeighbourhood(Bottom) = updateBoundaryWithObstacleSegment(cardinalNeighbourhood(Bottom), segment)
          }
        }
      }
    }

    for (obstacle <- continuousEnvCell.obstacles) {
      for (i <- 0 until obstacle.points) {
        val point = (obstacle.xs(i), obstacle.ys(i))
        if (isTopLeft(continuousEnvCell, point)) {
          diagonalNeighbourhood(TopLeft) = null
        } else if (isTopRight(continuousEnvCell, point)) {
          diagonalNeighbourhood(TopRight) = null
        } else if (isBottomRight(continuousEnvCell, point)) {
          diagonalNeighbourhood(BottomRight) = null
        } else if (isBottomLeft(continuousEnvCell, point)) {
          diagonalNeighbourhood(BottomLeft) = null
        }
      }
    }

    val newCellNeighbourhood = Neighbourhood(Map.from(cardinalNeighbourhood), Map.from(diagonalNeighbourhood))

    newCellNeighbourhood
  }

  private def updateBoundaryWithObstacleSegment(boundary: Boundary, segment: Segment): Boundary = {
    val boundaries: MutableMap[Segment, GridMultiCellId] = MutableMap.empty

    for ((existingSegment, neighbourId) <- boundary.boundaries) {
      if (existingSegment.a < segment.a && existingSegment.b > segment.b) {
        boundaries(Segment(existingSegment.a, segment.a)) = neighbourId
        boundaries(Segment(segment.b, existingSegment.b)) = neighbourId
      } else if (existingSegment.a < segment.a && existingSegment.b <= segment.b) {
        boundaries(Segment(existingSegment.a, segment.a)) = neighbourId
      } else if (existingSegment.a >= segment.a && existingSegment.b > segment.b) {
        boundaries(Segment(segment.b, existingSegment.b)) = neighbourId
      } else if (existingSegment.a >= segment.a && existingSegment.b <= segment.b) {
        // nothing
      }
    }

    Boundary(Map.from(boundaries))
  }
}
