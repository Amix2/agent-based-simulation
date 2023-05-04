package pl.edu.agh.xinuk.config

import io.jvm.uuid.UUID
import pl.edu.agh.xinuk.model.{ObstacleMessage, Signal}

case class Obstacle(xs: Array[Int], ys: Array[Int], points: Int, xsOrig: Array[Int], ysOrig: Array[Int]) {

  def Hash(x: Int): Int = {
    var out = ((x >> 16) ^ (x)) * 0x45d9f3b;
    out = ((out >> 16) ^ out) * 0x45d9f3b;
    out = (out >> 16) ^ out;
    return out;
  }

  def Hash(xs: Array[Int]): Int = {
    xs.foldLeft(xs.size)((xs, x) => Hash(x) + 0x9e3779b9 + (xs << 6) + (xs >> 2))
  }

  def GetUUID() : UUID = {
    new UUID(Hash(xsOrig), Hash(ysOrig))
  }
  def GenerateSignal(currentTime: Double): Signal = {
    Signal(0, Map(GetUUID() -> ObstacleMessage.createNew(xsOrig, ysOrig)));
  }
}

case class ObstacleSegment(a: (Int, Int), b: (Int, Int)) {
  def x: Double = b._1 - a._1

  def y: Double = b._2 - a._2
}
