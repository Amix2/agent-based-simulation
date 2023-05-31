package pl.edu.agh.continuous.env

import com.typesafe.scalalogging.LazyLogging
import pl.edu.agh.continuous.env.algorithm.{ContinuousEnvMetrics, ContinuousEnvWorldCreator, GeoKinMetrics, GeoKinPlanCreator, GeoKinPlanResolver}
import pl.edu.agh.continuous.env.common.ToVec2Conversions.SignalMapConversionExtensions
import pl.edu.agh.continuous.env.model.ContinuousEnvCell
import pl.edu.agh.xinuk.Simulation
import pl.edu.agh.xinuk.model.{AgentMessage, CellState, ObstacleMessage, Signal}
import pl.edu.agh.xinuk.model.grid.GridSignalPropagation

import java.awt.Color

object ContinuousEnvMain extends LazyLogging {
  private val configPrefix = "continuous-env"

  def main(args: Array[String]): Unit = {
    import pl.edu.agh.xinuk.config.ValueReaders._
    new Simulation(
      configPrefix,
      GeoKinMetrics.MetricHeaders,
      ContinuousEnvWorldCreator,
      GeoKinPlanCreator,
      GeoKinPlanResolver,
      GeoKinMetrics.empty,
      GridSignalPropagation.Bending,
      cellToColor
    ).start()
  }

  private def cellToColor: PartialFunction[CellState, Color] = {
    case cellState =>
      cellState.contents match {
        // case _ => Color.WHITE
        case continuousEnvCell: ContinuousEnvCell => cellToColorSign(cellState, continuousEnvCell)
      }
  }

  def ToColor(value: Double, sum: Int): Color =
  {
    if(value == 0)
      return new Color(0, 100, 200);
    val red =  if(value > sum) 255 else (value / sum) * 255;
    val green = if(value <= sum) 0 else if(value - sum > sum) 255 else ((value- sum) / sum) * 255;
    val blue =  if(value <= 2*sum) 0 else if(value - 2*sum > sum) 255 else ((value- 2*sum) / sum) * 255;
    new Color(red.intValue(), green.intValue(), blue.intValue());
  }


  private def cellToColorSign(cellState: CellState, continuousEnvCell: ContinuousEnvCell): Color = {
    var x = continuousEnvCell.gridMultiCellId.x
    var y = continuousEnvCell.gridMultiCellId.y

//    var x = continuousEnvCell.BaseCoordinates.x
//    var y = continuousEnvCell.BaseCoordinates.y
    return new Color(105,105,150)
    var count = 0;
    var maxVal = 0.0;

    val (agentMessages, obstacleMessages) = cellState.signalMap.toObjectMessagesSplit match {
      case (agents, obstacles) => (agents, obstacles)
      case _ => (List.empty[AgentMessage], List.empty[ObstacleMessage])
    }

    var messages = cellState.signalMap.toObjectMessages;
    messages.foreach({ case (id, sig) => {
      maxVal = math.max(maxVal, sig.GetDistanceToLive());
      count += 1;
    }
    })
    if (x == 10 && y == 1) {
    }
    if (x == 1 && y == 10) {
    }
    var sum = 3;
    //if (x == 1 && y == 12)
      //return new Color(0, 255, 255);
    return ToColor(count, sum)
    if (continuousEnvCell.initialSignal.value > 0) {
      Color.BLUE
    } else if (continuousEnvCell.visited) {
      new Color(0, 128, 0)
    } else {
      val maxSignal = cellState.signalMap
        .values
        .toSeq
        .sortBy(s => -Math.abs(s.value))
        .collectFirst({ case s: Signal => s.value })
        .getOrElse(0d)

      if (maxSignal > 0.75) {
        ///Color.WHITE
        new Color(64, 64, 255)
      } else if (maxSignal > 0.3) {
        //Color.WHITE
        new Color(128, 128, 255)
      } else if (maxSignal > 0.1) {
        //Color.WHITE
        new Color(192, 192, 255)
      } else if (maxSignal > 0) {
        //Color.WHITE
        new Color(224, 224, 255)
      } else {
        Color.WHITE
      }
    }
  }
}
