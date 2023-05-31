package pl.edu.agh.xinuk.config

import java.awt.Color

case class RunnerDefinition(x: Double, y: Double, radius: Double, maxSpeed: Double, tag: String
                            , color: Color = Color.getHSBColor(scala.util.Random.nextFloat(), 1, 1))

