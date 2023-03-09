package pl.edu.agh.xinuk.model

import pl.edu.agh.xinuk.config.XinukConfig
import pl.edu.agh.xinuk.model.continuous.NeighbourhoodState

trait SignalPropagation {
  def calculateUpdate(iteration: Long, neighbourhoodState: NeighbourhoodState, cell: Cell)(implicit config: XinukConfig): SignalMap
}
