package pl.edu.agh.xinuk.config

import io.jvm.uuid.UUID
import pl.edu.agh.xinuk.model.{ObstacleMessage, Signal}

case class Obstacle(xs: Array[Int], ys: Array[Int], points: Int) {

//  def GenerateSignal(currentTime: Double): Signal = {
//    Signal(0, Map(id -> ObstacleMessage.createNew(xs, ys, points)));
//  }
}

case class ObstacleSegment(a: (Int, Int), b: (Int, Int)) {
  def x: Double = b._1 - a._1

  def y: Double = b._2 - a._2
}
