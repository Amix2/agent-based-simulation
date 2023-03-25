package pl.edu.agh.xinuk.model.grid

import io.jvm.uuid.UUID
import pl.edu.agh.xinuk.config.XinukConfig
import pl.edu.agh.xinuk.model.continuous.NeighbourhoodState
import pl.edu.agh.xinuk.model.{AgentMessage, Cell, Direction, Signal, SignalMap, SignalPropagation}

object GridSignalPropagation {

  final val Standard: SignalPropagation = GridSignalPropagationStandard
  final val Bending: SignalPropagation = GridSignalPropagationBending

  @inline private def getPropagatedSignal(neighbourhoodState: NeighbourhoodState, neighbourDirection: Direction, signalDirection: Direction)
                                         (implicit config: XinukConfig): Signal = {
    neighbourDirection match {

      case cardinal@(GridDirection.Top | GridDirection.Right | GridDirection.Bottom | GridDirection.Left) =>
        neighbourhoodState.cardinalNeighbourhoodState(cardinal.asInstanceOf[GridDirection])
          .boundaryStates
          .map { case (segment, cellState) => Signal((segment.b - segment.a) / config.cellSize * cellState.signalMap(signalDirection).value, cellState.signalMap(signalDirection).objectMessages) }
          .foldLeft(Signal.zero)(_ + _)

      case diagonal@(GridDirection.TopLeft | GridDirection.TopRight | GridDirection.BottomRight | GridDirection.BottomLeft) =>
        if (neighbourhoodState.diagonalNeighbourhoodState.contains(diagonal.asInstanceOf[GridDirection]))
        {
          neighbourhoodState.diagonalNeighbourhoodState(diagonal.asInstanceOf[GridDirection])
            .signalMap(signalDirection)
        }
        else
        {
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
          .map { case (segment, cellState) => Signal((segment.b - segment.a) / config.cellSize * cellState.contents.generateSignal(iteration).value, cellState.contents.generateSignal(iteration).objectMessages) }
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
    def calculateUpdate(iteration: Long, neighbourhoodState: NeighbourhoodState, cell: Cell)(implicit config: XinukConfig): SignalMap = {
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

  private final object GridSignalPropagationBending extends SignalPropagation {
    def direct: Double = 0.42
    def adjacent: Double = 0.29

    def getSelfGeneratedSignal(cell: Cell, iteration: Long)(implicit config: XinukConfig): Signal =
      Signal(0, cell.state.contents.generateSignal(iteration).objectMessages)

    def calculateUpdate(iteration: Long, neighbourhoodState: NeighbourhoodState, cell: Cell)(implicit config: XinukConfig): SignalMap = {
      config.worldType.directions.map(direction => {
        val propagatedSignal = getPropagatedSignal(neighbourhoodState, direction, direction)
        val generatedSignal = getGeneratedSignal(neighbourhoodState, direction, iteration)
        val selfGeneratedSignal = getSelfGeneratedSignal(cell, iteration)
        val adjacentSignal = direction.adjacent.map { d => getPropagatedSignal(neighbourhoodState, direction, d) }.reduce(_ + _)
        (direction,
          propagatedSignal * direct
            +
            adjacentSignal * adjacent
            +
            generatedSignal
            +
            selfGeneratedSignal
        )
      }
      ).toMap
    }
  }

}
