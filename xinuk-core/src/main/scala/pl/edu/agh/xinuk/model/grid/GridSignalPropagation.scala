package pl.edu.agh.xinuk.model.grid

import pl.edu.agh.xinuk.config.XinukConfig
import pl.edu.agh.xinuk.model.continuous.NeighbourhoodState
import pl.edu.agh.xinuk.model.{Direction, Signal, SignalMap, SignalPropagation}

object GridSignalPropagation {

  final val Standard: SignalPropagation = GridSignalPropagationStandard
  /*final val Bending: SignalPropagation = GridSignalPropagationBending*/

  @inline private def getPropagatedSignal(neighbourhoodState: NeighbourhoodState, neighbourDirection: Direction, signalDirection: Direction)
                                         (implicit config: XinukConfig): Signal = {
    neighbourDirection match {
      case cardinal@(GridDirection.Top | GridDirection.Right | GridDirection.Bottom | GridDirection.Left) =>
        neighbourhoodState.cardinalNeighbourhoodState(cardinal.asInstanceOf[GridDirection])
          .boundaryStates
          .map { case (segment, cellState) => Signal((segment.b - segment.a) / config.cellSize * cellState.signalMap(signalDirection).value) }
          .foldLeft(Signal.zero)(_ + _)
      case diagonal@(GridDirection.TopLeft | GridDirection.TopRight | GridDirection.BottomRight | GridDirection.BottomLeft) =>
        if (neighbourhoodState.diagonalNeighbourhoodState.contains(diagonal.asInstanceOf[GridDirection])) {
          neighbourhoodState.diagonalNeighbourhoodState(diagonal.asInstanceOf[GridDirection])
            .signalMap(signalDirection)
        } else {
          Signal.zero
        }
      case _ => Signal.zero
    }
  }

  @inline private def getGeneratedSignal(neighbourhoodState: NeighbourhoodState, neighbourDirection: Direction, iteration: Long)
                                        (implicit config: XinukConfig): Signal = {
    neighbourDirection match {
      case cardinal@(GridDirection.Top | GridDirection.Right | GridDirection.Bottom | GridDirection.Left) =>
        neighbourhoodState.cardinalNeighbourhoodState(cardinal.asInstanceOf[GridDirection])
          .boundaryStates
          .map { case (segment, cellState) => Signal((segment.b - segment.a) / config.cellSize * cellState.contents.generateSignal(iteration).value) }
          .foldLeft(Signal.zero)(_ + _)
      case diagonal@(GridDirection.TopLeft | GridDirection.TopRight | GridDirection.BottomRight | GridDirection.BottomLeft) =>
        if (neighbourhoodState.diagonalNeighbourhoodState.contains(diagonal.asInstanceOf[GridDirection])) {
          neighbourhoodState.diagonalNeighbourhoodState(diagonal.asInstanceOf[GridDirection])
            .contents
            .generateSignal(iteration)
        } else {
          Signal.zero
        }
      case _ => Signal.zero
    }
  }

  private final object GridSignalPropagationStandard extends SignalPropagation {
    def calculateUpdate(iteration: Long, neighbourhoodState: NeighbourhoodState)(implicit config: XinukConfig): SignalMap = {
      config.worldType.directions.map({
        case cardinal@(GridDirection.Top | GridDirection.Right | GridDirection.Bottom | GridDirection.Left) =>
          (
            cardinal,
            cardinal.withAdjacent.map { d => getPropagatedSignal(neighbourhoodState, cardinal, d) }.reduce(_ + _) +
              getGeneratedSignal(neighbourhoodState, cardinal, iteration)
          )
        case diagonal@(GridDirection.TopLeft | GridDirection.TopRight | GridDirection.BottomRight | GridDirection.BottomLeft) =>
          (
            diagonal,
            getPropagatedSignal(neighbourhoodState, diagonal, diagonal) +
              getGeneratedSignal(neighbourhoodState, diagonal, iteration)
          )
        case direction => (direction, Signal.zero)
      }).toMap
    }
  }

  /*private final object GridSignalPropagationBending extends SignalPropagation {
    def direct: Double = 0.42
    def adjacent: Double = 0.29

    def calculateUpdate(iteration: Long, neighbourStates: Map[Direction, CellState])(implicit config: XinukConfig): SignalMap = {
      config.worldType.directions.map(direction =>
        (direction,
          getPropagatedSignal(neighbourStates, direction, direction) * direct +
            direction.adjacent.map { d => getPropagatedSignal(neighbourStates, direction, d) }.reduce(_ + _) * adjacent +
            getGeneratedSignal(neighbourStates, direction, iteration)
        )
      ).toMap
    }
  }*/

}
