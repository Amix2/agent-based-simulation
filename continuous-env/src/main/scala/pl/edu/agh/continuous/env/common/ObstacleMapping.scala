package pl.edu.agh.continuous.env.common

import pl.edu.agh.continuous.env.common.CellBoundedPosition.PositionExtensions
import pl.edu.agh.continuous.env.common.geometry.Line
import pl.edu.agh.continuous.env.model.ContinuousEnvCell
import pl.edu.agh.xinuk.algorithm.Vec2
import pl.edu.agh.xinuk.config.{Obstacle, ObstacleSegment}
import pl.edu.agh.xinuk.model.Direction

import java.util.UUID

object ObstacleMapping {
  implicit class NeighborContentsExtensions(val cells: Map[(ContinuousEnvCell, UUID), Direction]) extends AnyVal {

    def mapToObstacleLines(cellSize: Double): Iterable[Line] = {
      var lines: Iterable[Line] = Iterable.empty
      cells.foreach(cell => lines ++= mapCellToObstacleLines(cell._1._1.obstacles, Option(cell._2), cellSize))
      lines
    }
  }

  private def mapCellToObstacleLines(obstacles: Array[Obstacle], direction: Option[Direction], cellSize: Double): Iterable[Line] = {
    var lines: Iterable[Line] = Iterable.empty
    obstacles.map(obstacle => toObstacleSegments(obstacle))
      .map(segments => segments.map(segment =>
        if (direction.isEmpty) {
          Line(Vec2(segment._2.a._1, segment._2.a._2), Vec2(segment._2.b._1, segment._2.b._2))
        }
        else {
          Line(adjustObstacleVertex(segment._2.a._1, segment._2.a._2, direction.get, cellSize),
            adjustObstacleVertex(segment._2.b._1, segment._2.b._2, direction.get, cellSize))
        })
        .toIterable)
      .foreach(iterable => lines ++= iterable)
    lines
  }

  private def adjustObstacleVertex(x: Int, y: Int, direction: Direction, cellSize: Double): Vec2 = {
    val position: Vec2 = new Vec2(x, y)
    position.cellBounded(cellSize, true).adjust(direction, true)
  }

  def toObstacleSegments(obstacle: Obstacle): Array[(Int, ObstacleSegment)] = {
    var obstacleSegments: Array[(Int, ObstacleSegment)] = Array()
    for (i <- 0 until obstacle.points) {
      obstacleSegments = obstacleSegments :+ (i, ObstacleSegment((obstacle.xs(i), obstacle.ys(i)), (obstacle.xs((i + 1) % obstacle.points), obstacle.ys((i + 1) % obstacle.points))))
    }
    obstacleSegments
  }

  def Triangulation(measurements : List[Vec2]) : Seq[(Int, Int, Int)] = {
    case class ITRIANGLE(p1:Int, p2:Int, p3:Int)
    case class IEDGE(p1:Int, p2:Int)

    case class EdgeAnihilationSet(s: Set[IEDGE]) {
      def add(e: IEDGE): EdgeAnihilationSet = {
        if( s.contains(e) ) {
          printf(s"FOUND ${e} NOT REVERSED *********************\n")
          new EdgeAnihilationSet(s - e)
        }
        else {
          val e_reversed = IEDGE(e.p2, e.p1)
          if( s.contains(e_reversed) ) {
            //printf(s"removing ${e} reversed\n")
            new EdgeAnihilationSet(s - e_reversed)
          }
          else {
            //printf(s"adding ${e}\n")
            new EdgeAnihilationSet(s + e)
          }
        }
      }
      def toList() = s.toList
    }

    val EPSILON = 0.0000001

    //  Return TRUE if a point q(x,y) is inside the circumcircle made up of the points p1(x,y), p2(x,y), p3(x,y)
    //  The circumcircle centre (x,y) is returned and the radius r
    //  NOTE: A point on the edge is inside the circumcircle
    def CircumCircle( q:Vec2, p1:Vec2, p2:Vec2, p3:Vec2) : (/*inside :*/Boolean, /*center:*/Vec2, /*radius:*/Float) = {
      if ( Math.abs(p1.y-p2.y) < EPSILON && Math.abs(p2.y-p3.y) < EPSILON ) {
        System.err.println("CircumCircle: Points are colinear");
        println("CircumCircle: Points are colinear *****************************")
        (false, new Vec2(0,0), 0)
      }
      else {
        val mid1 = Vec2( (p1.x+p2.x)/2, (p1.y+p2.y)/2 )
        val mid2 = Vec2( (p2.x+p3.x)/2, (p2.y+p3.y)/2 )

        val c =
          if ( Math.abs(p2.y-p1.y) < EPSILON ) {
            //println("CircumCircle: p1&p2 have same y")
            val d2 = -(p3.x-p2.x) / (p3.y-p2.y)
            val xc =  mid1.x
            val yc =  d2 * (xc - mid2.x) + mid2.y
            new Vec2(xc, yc)
          }
          else
            if ( Math.abs(p3.y-p2.y) < EPSILON ) {
              //println("CircumCircle: p2&p3 have same y")
              val d1 = -(p2.x-p1.x) / (p2.y-p1.y)
              val xc =  mid2.x
              val yc =  d1 * (xc - mid1.x) + mid1.y
              new Vec2(xc, yc)
            }
            else {
              val d1 = -(p2.x-p1.x) / (p2.y-p1.y)
              val d2 = -(p3.x-p2.x) / (p3.y-p2.y)
              val xc =  ((d1*mid1.x - mid1.y) - (d2*mid2.x - mid2.y)) / (d1 - d2)
              val yc =  d1 * (xc - mid1.x) + mid1.y
              new Vec2(xc, yc)
            }

        val rsqr = {
          val (dx, dy) = (p2.x-c.x, p2.y-c.y)  // Distance from (any) 1 point on triangle to circle center
          dx*dx + dy*dy
        }
        val qsqr = {
          val (dx, dy) = (q.x -c.x, q.y -c.y)    // Distance from query_point to circle center
          dx*dx + dy*dy
        }

        ( qsqr <= rsqr, c, Math.sqrt(rsqr).toFloat )
      }
    }

    val n_points = measurements.length

    // Find the maximum and minimum vertex bounds, to allow calculation of the bounding triangle
    val Pmin = Vec2( measurements.map(_.x).min, measurements.map(_.y).min )  // Top Left
    val Pmax = Vec2( measurements.map(_.x).max, measurements.map(_.y).max )  // Bottom Right
    val diameter = (Pmax.x - Pmin.x) max (Pmax.y - Pmin.y)
    val Pmid = Vec2( (Pmin.x + Pmax.x)/2, (Pmin.y + Pmax.y)/2 )

    /*
      Set up the supertriangle, which is a triangle which encompasses all the sample points.
      The supertriangle coordinates are added to the end of the vertex list.
      The supertriangle is the first triangle in the triangle list.
    */

    val point_list = measurements ::: List(
      Vec2( Pmid.x - 2*diameter, Pmid.y - 1*diameter),
      Vec2( Pmid.x - 0*diameter, Pmid.y + 2*diameter),
      Vec2( Pmid.x + 2*diameter, Pmid.y - 1*diameter)
    )

    val main_current_triangles   = List( ITRIANGLE(n_points+0, n_points+1, n_points+2) ) // initially containing the supertriangle
    val main_completed_triangles: List[ITRIANGLE] = Nil                                  // initially empty

    def convert_relevant_triangles_into_new_edges(completed_triangles: List[ITRIANGLE], triangles: List[ITRIANGLE], point: Vec2) =
    //: (updated_completed: List[ITRIANGLE], updated_triangles: List[ITRIANGLE], edges:Set[IEDGE]) =
      triangles.foldLeft( (completed_triangles: List[ITRIANGLE], List[ITRIANGLE](), EdgeAnihilationSet(Set.empty[IEDGE])) ) {
        case ((completed, current, edges), triangle) => {
          // If the point 'point_being_added' lies inside the circumcircle then the three edges
          // of that triangle are added to the edge buffer and that triangle is removed.

          // Find the coordinates of the points in this incomplete triangle
          val corner1 = point_list( triangle.p1 )
          val corner2 = point_list( triangle.p2 )
          val corner3 = point_list( triangle.p3 )

          val (inside, circle, r) = CircumCircle(point,  corner1,  corner2,  corner3)

          // have we moved too far in x to bother with this one ever again? (initial point list must be sorted for this to work)
          if (circle.x + r < point.x) {  // (false &&) to disable the 'completed' optimisation
            //printf(s"point_x=${point.x} BEYOND triangle : ${triangle} with circle=[${circle}, ${r}]\n")
            ( triangle::completed, current, edges ) // Add this triangle to the 'completed' accumulator, and don't add it on current list
          }
          else {
            if(inside) {
              //printf(s"point INSIDE triangle : ${triangle}\n")
              // Add the triangle's edge onto the edge pile, and remove the triangle
              val edges_with_triangle_added =
              edges
                .add( IEDGE(triangle.p1, triangle.p2) )
                .add( IEDGE(triangle.p2, triangle.p3) )
                .add( IEDGE(triangle.p3, triangle.p1) )
              ( completed, current, edges_with_triangle_added )
            }
            else {
              //printf(s"point outside triangle : ${triangle}\n")
              ( completed, triangle::current, edges )  // This point was not inside this triangle - just add it to the 'current' list
            }
          }
        }
      }

    def update_triangle_list_for_new_point(completed_triangles: List[ITRIANGLE], triangles: List[ITRIANGLE], point_i: Int) = {
      // : (completed_triangles: List[ITRIANGLE], current_triangles: List[ITRIANGLE])
      val (completed_triangles_updated, current_triangles_updated, edges_created) =
        convert_relevant_triangles_into_new_edges(completed_triangles, triangles, point_list(point_i))

      // Form new triangles for the current point, all edges arranged in clockwise order.
      val new_triangles = for ( e <- edges_created.toList() ) yield ITRIANGLE( e.p1, e.p2, point_i )
      (completed_triangles_updated, new_triangles ::: current_triangles_updated)
    }

    // Go through points in x ascending order.  No need to sort the actual points, just output the point_i in correct sequence
    // (relies on sortBy being 'stable' - so that sorting on y first will enable duplicate detection afterwards)
    val points_sorted_xy_ascending = point_list.take(n_points).zipWithIndex sortBy(_._1.y) sortBy(_._1.x) map { case (Vec2(x,y), i) => i }
    //for ( i <- points_sorted_x_ascending ) printf(f"${i}%2d = [${point_list(i)}]\n")
    //printf(s"points_sorted_x_ascending = ${points_sorted_x_ascending}\n")

    val points_sorted_and_deduped =
      points_sorted_xy_ascending.foldLeft( (Nil:List[Int], -1) ) {
        case ((list, point_last), point_i) => if(point_last>=0 && point_list(point_last) == point_list(point_i)) {
          printf(s"Skipping duplicate points {${point_last},${point_i}}\n")
          ( list, point_i ) // Don't add this point to the list
        }
        else
          ( point_i::list, point_i )
      }._1.reverse // list of points is first element of tuple, and were pre-pended, so list needs reversing

    // Add each (original) point, one at a time, into the existing mesh
    val (final_completed, final_triangles) =
      points_sorted_and_deduped.foldLeft( (main_completed_triangles, main_current_triangles) ) {
        case ((completed, current), point_i) => update_triangle_list_for_new_point(completed, current, point_i)
      }

    val full_list_of_triangles = (final_completed ::: final_triangles)
    // filter out triangles with points that have point_i >= n_points (since these are part of the fake supertriangle)
    full_list_of_triangles.filterNot( t => (t.p1>=n_points || t.p2>=n_points || t.p3>=n_points)) map { t => (t.p1, t.p2, t.p3) }
  }
}
