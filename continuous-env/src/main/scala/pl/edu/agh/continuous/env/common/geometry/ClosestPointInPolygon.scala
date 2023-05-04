package pl.edu.agh.continuous.env.common.geometry

import pl.edu.agh.xinuk.algorithm.Vec2

object ClosestPointInPolygon {
  def distance(p1: Vec2, p2: Vec2): Double = {
    math.sqrt(math.pow(p1.x - p2.x, 2) + math.pow(p1.y - p2.y, 2))
  }

  def distanceToLine(p: Vec2, line: Line): Double = {
    val numerator = math.abs((line.p2.y - line.p1.y) * p.x - (line.p2.x - line.p1.x) * p.y + line.p2.x * line.p1.y - line.p2.y * line.p1.x)
    val denominator = distance(line.p1, line.p2)
    numerator / denominator
  }

  def closestPoint(p: Vec2, line: Line): Vec2 = {
    val u = ((p.x - line.p1.x) * (line.p2.x - line.p1.x) + (p.y - line.p1.y) * (line.p2.y - line.p1.y)) / math.pow(distance(line.p1, line.p2), 2)
    val closest = Vec2(line.p1.x + u * (line.p2.x - line.p1.x), line.p1.y + u * (line.p2.y - line.p1.y))
    closest
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
    polygons.map(poly => closestPointInPolygon(A, poly)).minBy(p => (p - A).length)
  }
}