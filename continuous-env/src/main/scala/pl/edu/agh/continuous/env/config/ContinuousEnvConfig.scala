package pl.edu.agh.continuous.env.config

import pl.edu.agh.xinuk.config.{Obstacle, GuiType, XinukConfig}
import pl.edu.agh.xinuk.model.{Signal, WorldType}

case class SphConfig(targetDensity: Double = 150
                     , stiffness: Double = 10000
                     , viscosity: Double = 100
                     , kernelSize: Double = 1.5)

final case class ContinuousEnvConfig(
                                worldType: WorldType,
                                worldWidth: Int,
                                worldHeight: Int,
                                iterationsNumber: Long,
                                deltaTime: Double,

                                signalSuppressionFactor: Double,
                                signalAttenuationFactor: Double,
                                signalSpeedRatio: Int,

                                workersRoot: Int,
                                isSupervisor: Boolean,
                                shardingMod: Int,

                                guiType: GuiType,
                                guiCellSize: Int,

                                personUnitAcceleration: Double,
                                personMinStepLength: Double,

                                cellSize: Int,


                                initialSignal: Signal,

                                maxAgentRadius: Double,

                                obstacles: List[Obstacle],

                                sphConfig: SphConfig
                              ) extends XinukConfig
