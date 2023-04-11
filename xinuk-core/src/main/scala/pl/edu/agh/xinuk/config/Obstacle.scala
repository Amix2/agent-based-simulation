package pl.edu.agh.xinuk.config

import io.jvm.uuid.UUID
import pl.edu.agh.xinuk.model.{ObstacleMessage, Signal}

case class Obstacle(xs: Array[Int], ys: Array[Int], points: Int, xsOrig: Array[Int], ysOrig: Array[Int]) {

  def Hash (x : Int) : Int = {
    var out = ((x >> 16) ^ (x+1)) * 0x45d9f3b;
    out = ((out >> 16) ^ out) * 0x45d9f3b;
    out = (out >> 16) ^ out;
    return out;
  }
  def GetUUID() : UUID = {
    var i1 = Hash(1)
    var i2 = Hash(2)
    xsOrig.foreach(i => i1 = i1 ^ Hash(i));
    ysOrig.foreach(i => i2 = i2 ^ Hash(i));
    new UUID(i1, i2)
  }
  def GenerateSignal(currentTime: Double): Signal = {
    Signal(0, Map(GetUUID() -> ObstacleMessage.createNew(xsOrig, ysOrig, points)));
  }
}

case class ObstacleSegment(a: (Int, Int), b: (Int, Int)) {
  def x: Double = b._1 - a._1

  def y: Double = b._2 - a._2
}
