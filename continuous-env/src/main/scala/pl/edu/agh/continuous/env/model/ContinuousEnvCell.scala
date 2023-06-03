package pl.edu.agh.continuous.env.model

import pl.edu.agh.xinuk.algorithm.MathUtils.DoubleExtensions
import pl.edu.agh.continuous.env.common.geometry.Line
import pl.edu.agh.continuous.env.config.ContinuousEnvConfig
import pl.edu.agh.continuous.env.model.continuous.CellOutline
import pl.edu.agh.xinuk.algorithm.Vec2
import pl.edu.agh.xinuk.config.{Obstacle, XinukConfig}
import pl.edu.agh.xinuk.model.continuous.{GridMultiCellId, Neighbourhood}
import pl.edu.agh.xinuk.model.{CellContents, Signal}

final case class ContinuousEnvCell(initialSignal: Signal, gridMultiCellId: GridMultiCellId)(implicit config: ContinuousEnvConfig) extends CellContents {
  override def generateSignal(iteration: Long)(implicit config: XinukConfig): Signal =
    { // runners can generate signal in cell they are standing on
      var signals = (List(initialSignal)
        ++ runners.map(r => r.GenerateSignal(iteration * config.deltaTime, this))
        ++ globalObstacles.map(r => r.GenerateSignal(iteration * config.deltaTime))
        )
        .foldLeft(Signal.zero)(_ + _);
      signals
    }

  override def getRunnerCount = runners.size;

  override def signalFactor(iteration: Long)
                           (implicit config: XinukConfig): Double = {
    ((totalCellField() - totalRunnersField) / totalCellField() * 0.5)
      .pow(10.0)
      .clip(lowerBound = 0.0000001, upperBound = 1.0)
  }

  private def totalCellField(): Double =
    cellOutline.width * cellOutline.height

  private def totalRunnersField: Double = runners.map(_.fakeMass).sum
  def BaseCoordinates(): Vec2 = Vec2(gridMultiCellId.y * cellOutline.width, gridMultiCellId.x * cellOutline.height);

  var cellOutline: CellOutline = CellOutline.default()
  var neighbourhood: Neighbourhood = Neighbourhood.empty()
  var obstacles: Array[Obstacle] = Array()
  var globalObstacles: Array[Obstacle] = Array()
  var runners: Array[Runner] = Array()
  var graph: Map[Vec2, Set[Vec2]] = Map.empty
  var cardinalSegments: Map[Line, GridMultiCellId] = Map.empty
  var generation: Long = 0
  var visited = false
}
