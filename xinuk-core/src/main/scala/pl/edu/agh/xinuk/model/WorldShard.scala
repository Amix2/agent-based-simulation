package pl.edu.agh.xinuk.model

import pl.edu.agh.xinuk.config.XinukConfig
import pl.edu.agh.xinuk.model.continuous.{Boundary, BoundaryState, Neighbourhood, NeighbourhoodState}


trait WorldType {
  def directions: Seq[Direction]
}

trait WorldShard {
  def cells: Map[CellId, Cell]

  def localCellIds: Set[CellId]

  def cellNeighbours: Map[CellId, Neighbourhood]

  def workerId: WorkerId

  def outgoingCells: Map[WorkerId, Set[CellId]]

  def incomingCells: Map[WorkerId, Set[CellId]]

  def outgoingWorkerNeighbours: Set[WorkerId] = outgoingCells.keySet + workerId

  def incomingWorkerNeighbours: Set[WorkerId] = incomingCells.keySet + workerId

  def cellToWorker: Map[CellId, WorkerId]

  def calculateSignalUpdates(iteration: Long, signalPropagation: SignalPropagation)(implicit config: XinukConfig): Map[CellId, SignalMap] = {
    cells.map { cellId_Cell =>
      (cellId_Cell._1, signalPropagation.calculateUpdate(iteration, toNeighbourhoodState(cellId_Cell._1), cellId_Cell._2))
    }
  }.toMap

  def toNeighbourhoodState(cellId: CellId)(implicit config: XinukConfig): NeighbourhoodState = {
    val cardinalNeighbourhoodState = cellNeighbours(cellId)
      .cardinalNeighbourhood
      .map { case (direction, boundary) => (direction, toBoundaryState(boundary)) }
    val diagonalNeighbourhoodState = cellNeighbours(cellId)
      .diagonalNeighbourhood
      .map { case (direction, neighbourId) =>
        if (cells.contains(neighbourId)) {
          (direction, cells(neighbourId).state)
        } else {
          (direction, CellState.empty())
        }
      }

    NeighbourhoodState(cardinalNeighbourhoodState, diagonalNeighbourhoodState)
  }

  private def toBoundaryState(boundary: Boundary): BoundaryState = {
    val segmentStates = boundary.boundaries
      .map { case (segment, neighbourId) => (segment, cells(neighbourId).state) }
    BoundaryState.of(segmentStates)
  }
}


trait WorldBuilder {
  def apply(cellId: CellId): Cell

  def update(cellId: CellId, cellState: CellState): Unit

  def connectOneWay(from: CellId, direction: Direction, to: CellId): Unit

  final def connectTwoWay(from: CellId, direction: Direction, to: CellId): Unit = {
    connectOneWay(from, direction, to)
    connectOneWay(to, direction.opposite, from)
  }

  def build(): Map[WorkerId, WorldShard]
}
