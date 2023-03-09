package pl.edu.agh.xinuk.model.continuous

import pl.edu.agh.xinuk.model.{CellState}
import pl.edu.agh.xinuk.model.grid.GridDirection
import pl.edu.agh.xinuk.model.grid.GridDirection.{Bottom, BottomLeft, BottomRight, Left, Right, Top, TopLeft, TopRight}

case class Neighbourhood(cardinalNeighbourhood: Map[GridDirection, Boundary],
                         diagonalNeighbourhood: Map[GridDirection, GridMultiCellId])

object Neighbourhood {
  private val Empty: Neighbourhood = Neighbourhood.apply(
    Map(
      Top -> Boundary.empty(),
      Right -> Boundary.empty(),
      Bottom -> Boundary.empty(),
      Left -> Boundary.empty()
    ),
    Map(
      TopLeft -> null,
      TopRight -> null,
      BottomRight -> null,
      BottomLeft -> null
    ))

  def empty(): Neighbourhood = return Neighbourhood.Empty
}

case class NeighbourhoodState(cardinalNeighbourhoodState: Map[GridDirection, BoundaryState],
                              diagonalNeighbourhoodState: Map[GridDirection, CellState])
