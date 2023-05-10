package pl.edu.agh.continuous.env.common.geometry

import pl.edu.agh.xinuk.algorithm.Vec2

object ClosestPointInPolygon {
  def distance(p1: Vec2, p2: Vec2): Double = {
    math.sqrt(math.pow(p1.x - p2.x, 2) + math.pow(p1.y - p2.y, 2))
  }

  def distanceToLine(p: Vec2, line: Line): Double = {
    line.segmentDistance(p)
  }

  def closestPoint(P: Vec2, line: Line): Vec2 = {
    val AB = line.p2 - line.p1
    val AP = P - line.p1
    val lengthSqrAB = AB.x * AB.x + AB.y * AB.y
    var t = (AP.x * AB.x + AP.y * AB.y) / lengthSqrAB
    if (t < 0)
      t = 0;
    if (t > 1)
      t = 1;
    return line.p1 + AB * t
  }




  def closestPointInPolygon(A: Vec2, polygon: List[Vec2]): Vec2 = {
    var minDistance = distance(A, polygon.head)
    var closest = polygon.head
    for (i <- polygon.indices) {
      val edge = Line(polygon(i), if (i == polygon.length - 1) polygon.head else polygon(i + 1))
      val p = closestPoint(A, edge)
      val d = distanceToLine(A, edge)
      if (d < minDistance) {
        minDistance = d
        closest = p
      }
    }
    closest
  }

  def closestPointInPolygons(A: Vec2, polygons: List[List[Vec2]]): Vec2 = {
    if(polygons.isEmpty)
      return Vec2(Float.MaxValue,Float.MaxValue);
    polygons.map(poly => closestPointInPolygon(A, poly)).minBy(p => (p - A).length)
  }
}