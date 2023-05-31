package pl.edu.agh.continuous.env.algorithm

import com.fasterxml.jackson.module.scala.deser.overrides.MutableList
import pl.edu.agh.continuous.env.common.ToVec2Conversions.SignalMapConversionExtensions
import pl.edu.agh.continuous.env.config.ContinuousEnvConfig
import pl.edu.agh.continuous.env.model.{ContinuousEnvCell, Runner}
import pl.edu.agh.xinuk.algorithm.Vec2
import pl.edu.agh.xinuk.model.{AgentMessage, Direction, ObstacleMessage, SignalMap}

import scala.collection.mutable.ListBuffer
import java.util.{NoSuchElementException, UUID}
import scala.util.control.Breaks.{break, breakable}


//  BASED ON https://gamma.cs.unc.edu/RVO2/downloads/

object ORCAVelocityCalculator {

  case class ObstacleSegment(distSq: Double, obs1Point: Vec2, obs2Point: Vec2,
                             obs1Direction: Vec2, obs2Direction: Vec2,
                             leftNeighborDirection: Vec2) extends Comparable[ObstacleSegment] {
    def compareTo(other: ObstacleSegment): Int = {
      distSq.compareTo(other.distSq)
    }
  }

  case class OtherAgent(distSq: Double, position: Vec2, velocity: Vec2, radius: Double) extends Comparable[OtherAgent] {
    def compareTo(other: OtherAgent): Int = {
      distSq.compareTo(other.distSq)
    }
  }

  def distSqPointLineSegment(lineP1: Vec2, lineP2: Vec2, point: Vec2): Double = {
    val lineVec = lineP2 - lineP1
    val r = (point - lineP1).dot(lineVec) / lineVec.lengthSq

    if (r < 0.0f) {
      (point - lineP1).lengthSq
    } else if (r > 1.0f) {
      (point - lineP2).lengthSq
    } else {
      val projection = lineP1 + (lineVec * r)
      (point - projection).lengthSq
    }
  }

  case class Line(direction: Vec2, point: Vec2) {
    def isNan: Boolean = {
      direction.x.isNaN || direction.y.isNaN || point.x.isNaN || point.y.isNaN
    }
  }

  case class Vec2x2(c0: Vec2, c1: Vec2) {  }

  def determinant(m: Vec2x2): Double = {
    val a = m.c0.x
    val b = m.c1.x
    val c = m.c0.y
    val d = m.c1.y
    a * d - b * c
  }

  def adjustVelocityForRunner(runner: Runner,
                         signalMap: SignalMap,
                         cell: ContinuousEnvCell,
                         neighbourContents: Map[(ContinuousEnvCell, UUID), Direction],
                         config: ContinuousEnvConfig): Runner = {

    val fAgentPOV = math.Pi
    val fAgentPOVCos = math.cos(fAgentPOV)
    val fTimeHorison = 5 // [s]

    val fTimeHorisonObs = 5
    val invTimeHorizon = 1.0f / fTimeHorison
    val invTimeHorizonObst = 1.0f / fTimeHorisonObs
    val invTimeStep = 1.0f / config.deltaTime

    val (agentMessages, obstacleMessages) = signalMap.toObjectMessagesSplit match {
      case (agents, obstacles) => (agents, obstacles)
      case _ => (List.empty[AgentMessage], List.empty[ObstacleMessage])
    }

    val myPos = (runner.globalCellPosition(config) + cell.BaseCoordinates(config)) * 0.01;
    var myVel = runner.velocity * 0.01;
    myVel = Vec2(myVel.x, -myVel.y);
    val myRad = runner.radius * 0.01

    val obstaclePolygons = obstacleMessages.map(msg => msg.ToVec2List);

    val obstacleSegmentsCreator = ListBuffer.empty[ObstacleSegment];

    for(obstacle <- obstaclePolygons)
      for(i <- obstacle.indices)
        {
          val nextI = (i + 1) % obstacle.size;
          val prevI = (i - 1 + obstacle.size) % obstacle.size;
          val nextNextI = (nextI + 1) % obstacle.size;
          val obs1Point = obstacle(i) * 0.01
          val obs2Point = obstacle(nextI) * 0.01
          val obs1Direction = (obstacle(nextI) - obstacle(i)).normalized
          val obs2Direction = (obstacle(nextNextI) - obstacle(nextI)).normalized
          val leftNeighborDir = (obstacle(nextI) - obstacle(prevI)).normalized
          val distSq = distSqPointLineSegment(obs1Point, obs2Point, myPos)
          if(obs1Point != obs2Point)
            obstacleSegmentsCreator.addOne(ObstacleSegment(distSq, obs1Point, obs2Point, obs1Direction, obs2Direction, leftNeighborDir))
        }
    var  obstacleSegments = obstacleSegmentsCreator.sorted.toList

    val agentsCreator = ListBuffer.empty[OtherAgent];
    for(agent <- agentMessages)
      {
        val otherPos = agent.pos* 0.01
        if((otherPos - myPos).length > myRad/2)
          {
            var otherVel = agent.vel* 0.01;
            otherVel = Vec2(otherVel.x, -otherVel.y);

            val otherRad = agent.radius* 0.01;

            val distSq = (otherPos - myPos).lengthSq;
            agentsCreator.addOne( OtherAgent(distSq, otherPos, otherVel, otherRad));

          }

      }

    var  otherAgents = agentsCreator.sorted.toList

    val orcaLinesCreator = ListBuffer.empty[Line];

    for(obstacleSegment <- obstacleSegments) {
      breakable
      {
        var obs1Point = obstacleSegment.obs1Point
        var obs2Point = obstacleSegment.obs2Point

        var obs1Direction = obstacleSegment.obs1Direction
        var obs2Direction = obstacleSegment.obs2Direction

        val relativePosition1 = obs1Point - myPos
        val relativePosition2 = obs2Point - myPos


        /*
         * Check if velocity obstacle of obstacle is already taken care
         * of by previously constructed obstacle ORCA lines.
         */
        var alreadyCovered = false;
        for(line <- orcaLinesCreator if !alreadyCovered)
        {
          val f1 = relativePosition1 * invTimeHorizonObst - line.point
          val f2 = relativePosition2 * invTimeHorizonObst- line.point
          val lineDir = line.direction
          if (determinant(Vec2x2(f1, lineDir)) - invTimeHorizonObst * myRad >= -RVO_EPSILON
            && determinant(Vec2x2(f2, lineDir)) - invTimeHorizonObst * myRad >= -RVO_EPSILON)
            alreadyCovered = true
        }

        if(alreadyCovered)
          break();

        /* Not yet covered. Check for collisions. */
        val distSq1 = relativePosition1.lengthSq
        val distSq2 = relativePosition2.lengthSq

        val radiusSq = myRad * myRad

        val obstacleVector = obs2Point - obs1Point
        val s = dot(-relativePosition1, obstacleVector) / obstacleVector.lengthSq
        val distSqLine = (-relativePosition1 - obstacleVector * s).lengthSq

        if (s < 0.0f && distSq1 <= radiusSq)
        {
          /* Collision with left vertex. Ignore if non-convex. */
          if (true /*obstacle1.convex_*/ )
          {
            val point = Vec2(0.0f, 0.0f)
            val direction = Vec2(-relativePosition1.y, relativePosition1.x).normalized
            orcaLinesCreator.addOne(Line(direction, point));

          }
          break();

        }
        else if (s > 1.0f && distSq2 <= radiusSq)
        {
          /*
          * Collision with right vertex. Ignore if non-convex or if
          * it will be taken care of by neighboring obstacle.
          */
          if (determinant(Vec2x2(relativePosition2, obs2Direction)) >= 0.0f)  // && obstacle2.convex_
          {
            val point = Vec2(0.0f, 0.0f)
            val direction = Vec2(-relativePosition2.y, relativePosition2.x).normalized
            orcaLinesCreator.addOne(Line(direction, point));
          }
          break();

        }
        else if (s >= 0.0f && s <= 1.0f && distSqLine <= radiusSq)
        {
          /* Collision with obstacle segment. */
          val point = Vec2(0.0f, 0.0f)
          val direction = -obs1Direction
          orcaLinesCreator.addOne(Line(direction, point));
          break();
        }
        /*
         * No collision. Compute legs. When obliquely viewed, both legs
         * can come from a single vertex. Legs extend cut-off line when
         * non-convex vertex.
         */
        var leftLegDirection: Vec2 = null;
        var rightLegDirection: Vec2 = null;
        if (s < 0.0f && distSqLine <= radiusSq)
        {
          /*
          * Obstacle viewed obliquely so that left vertex
          * defines velocity obstacle.
          */

          obs2Point = obs1Point
          obs2Direction = obs1Direction
          val leg1 = math.sqrt(distSq1 - radiusSq)
          leftLegDirection = Vec2(relativePosition1.x * leg1 - relativePosition1.y * myRad, relativePosition1.x * myRad + relativePosition1.y * leg1) / distSq1
          rightLegDirection = Vec2(relativePosition1.x * leg1 + relativePosition1.y * myRad, -relativePosition1.x * myRad + relativePosition1.y * leg1) / distSq1
        }
        else if (s > 1.0f && distSqLine <= radiusSq)
        {
          /*
            * Obstacle viewed obliquely so that
            * right vertex defines velocity obstacle.
            */
          obs1Point = obs2Point
          obs1Direction = obs2Direction
          val leg2 = math.sqrt(distSq2 - radiusSq)
          leftLegDirection = Vec2(relativePosition2.x * leg2 - relativePosition2.y * myRad, relativePosition2.x * myRad + relativePosition2.y * leg2) / distSq2
          rightLegDirection = Vec2(relativePosition2.x * leg2 + relativePosition2.y * myRad, -relativePosition2.x * myRad + relativePosition2.y * leg2) / distSq2
        }
        else
        {
          /* Usual situation. */
          if (true /*obstacle1.convex_*/ )
          {
            val leg1 = math.sqrt(distSq1 - radiusSq)
            leftLegDirection = Vec2(relativePosition1.x * leg1 - relativePosition1.y * myRad, relativePosition1.x * myRad + relativePosition1.y * leg1) / distSq1
          }
          //else
          //    /* Left vertex non-convex; left leg extends cut-off line. */
          //    leftLegDirection = -obs1Direction;
          if (true /*obstacle2.convex_*/ )
          {
            val leg2 = math.sqrt(distSq2 - radiusSq)
            rightLegDirection = Vec2(relativePosition2.x * leg2 + relativePosition2.y * myRad, -relativePosition2.x * myRad + relativePosition2.y * leg2) / distSq2
          }
          //    /* Right vertex non-convex; right leg extends cut-off line. */
          //    rightLegDirection = obs1Direction;
        }

        /*
        * Legs can never point into neighboring edge when convex
        * vertex, take cutoff-line of neighboring edge instead. If
        * velocity projected on "foreign" leg, no constraint is added.
        */

        val leftNeighborDir = obstacleSegment.leftNeighborDirection

        var isLeftLegForeign = false
        var isRightLegForeign = false

        if (determinant(Vec2x2(leftLegDirection, -leftNeighborDir)) >= 0.0f)  // && obstacle1.convex_
        {
          /* Left leg points into obstacle. */
          leftLegDirection = -leftNeighborDir
          isLeftLegForeign = true
        }

        if (determinant(Vec2x2(rightLegDirection, obs2Direction)) <= 0.0f)  // && obstacle2.convex_
        {
          /* Right leg points into obstacle. */
          rightLegDirection = obs2Direction
          isRightLegForeign = true
        }

        /* Compute cut-off centers. */
        val leftCutOff =  (obs1Point - myPos) * invTimeHorizonObst
        val rightCutOff =  (obs2Point - myPos) * invTimeHorizonObst
        val cutOffVector = rightCutOff - leftCutOff

        /* Project current velocity on velocity obstacle. */

        /* Check if current velocity is projected on cutoff circles. */
        val t = if (obs1Point == obs2Point) 0.5f
                  else dot((myVel - leftCutOff), cutOffVector) / cutOffVector.lengthSq
        val tLeft = dot(myVel - leftCutOff, leftLegDirection)
        val tRight = dot(myVel - rightCutOff, rightLegDirection)

        if ((t < 0.0f && tLeft < 0.0f) || (obs1Point == obs2Point && tLeft < 0.0f && tRight < 0.0f))
        {
          /* Project on left cut-off circle. */
          val unitW = (myVel - leftCutOff).normalized
          val direction = Vec2(unitW.y, -unitW.x)
          val point = leftCutOff + unitW * myRad * invTimeHorizonObst
          orcaLinesCreator.addOne(Line(direction, point));
          break();
        }
        else if (t > 1.0f && tRight < 0.0f)
        {
          /* Project on right cut-off circle. */
          val unitW = (myVel - rightCutOff).normalized
          val direction = Vec2(unitW.y, -unitW.x)
          val point = rightCutOff + unitW * myRad * invTimeHorizonObst
          orcaLinesCreator.addOne(Line(direction, point));
          break();
        }

        /*
         * Project on left leg, right leg, or cut-off line, whichever is
         * closest to velocity.
         */
        val distSqCutoff = if(t < 0.0f || t > 1.0f || (obs1Point == obs2Point)) Float.PositiveInfinity else (myVel - (leftCutOff + cutOffVector * t )).lengthSq;

        val distSqLeft = if(tLeft < 0.0f) Float.PositiveInfinity else (myVel - (leftCutOff + leftLegDirection * tLeft)).lengthSq;

        val distSqRight = if(tRight < 0.0f) Float.PositiveInfinity else (myVel - (rightCutOff + rightLegDirection * tRight )).lengthSq;

        if (distSqCutoff <= distSqLeft && distSqCutoff <= distSqRight)
        {
          /* Project on cut-off line. */  // //////////////////////////
          val direction = -obs1Direction;
          val point = leftCutOff + Vec2(-direction.y, direction.x) * myRad * invTimeHorizonObst ;
          orcaLinesCreator.addOne(Line(direction, point));
          break();
        }

        if (distSqLeft <= distSqRight)
        {
          /* Project on left leg. */
          if (isLeftLegForeign)
            break();

          val direction = leftLegDirection;
          val point = leftCutOff + Vec2(-direction.y, direction.x) * myRad * invTimeHorizonObst;
          orcaLinesCreator.addOne(Line(direction, point));
          break();
        }

        /* Project on right leg. */
        if (isRightLegForeign)
          break();

        var direction = -rightLegDirection;
        var point = rightCutOff + Vec2(-direction.y, direction.x) * myRad * invTimeHorizonObst;
        orcaLinesCreator.addOne(Line(direction, point));
      }
    }

    val numObstLines = orcaLinesCreator.size


      for(other <- otherAgents)
      {
        val otherPos = other.position
        val otherVel = other.velocity
        val otherRad = other.radius

        val relativePosition = otherPos - myPos
        val relativeVelocity = myVel - otherVel
        val distSq = relativePosition.lengthSq
        val combinedRadius = myRad + otherRad
        val combinedRadiusSq = combinedRadius * combinedRadius


        var u = Vec2(0, 0);  // just temp values
        var lineDir : Vec2 = Vec2(0, 0);
        var linePoint : Vec2 = Vec2(0, 0);
        if (distSq > combinedRadiusSq)
        { // no collision
          // Vector from cutoff center to relative velocity.
          val w = relativeVelocity - relativePosition * invTimeHorizon
          val wLengthSq = w.lengthSq
          val dotProduct1 = dot(w, relativePosition)
          if (dotProduct1 < 0.0f && dotProduct1 * dotProduct1 > combinedRadiusSq * wLengthSq) { // project on cut-off circle
            val wLength = math.sqrt(wLengthSq)
            val unitW = w / wLength
            lineDir = Vec2(unitW.y, -unitW.x) // perpendicular

            u = unitW * (combinedRadius * invTimeHorizon - wLength)
          }
          else { // project on legs
            val leg = math.sqrt(distSq - combinedRadiusSq)
            if (determinant(Vec2x2(relativePosition, w)) > 0) { // project on left leg
              lineDir = Vec2(relativePosition.x * leg - relativePosition.y * combinedRadius, relativePosition.x * combinedRadius + relativePosition.y * leg) / distSq
            }
            else { // project on right leg
              lineDir = -(Vec2(relativePosition.x * leg + relativePosition.y * combinedRadius, -(relativePosition.x) * combinedRadius + relativePosition.y * leg)) / distSq
            }
            val dotProduct2 = dot(relativeVelocity, lineDir)
            u =  lineDir * dotProduct2 - relativeVelocity
          }
        }
        else { // collision, project on cit-off circle of time timeStep
          val w = relativeVelocity - relativePosition * invTimeStep
          val wLength = w.length
          val unitW = w / wLength
          lineDir = Vec2(unitW.y, -(unitW.x))
          u = unitW * (combinedRadius * invTimeStep - wLength)
        }
        linePoint = myVel + u *  0.5f
        orcaLinesCreator.addOne(Line(Vec2(lineDir.x, lineDir.y), linePoint));

      }

    val orcaLines = orcaLinesCreator.toList;
    if(orcaLines.isEmpty)
      return runner;
    var (lineFail, newVel) = LinearProgram2(orcaLines, runner.maxSpeed*0.01, myVel, false);
    if (lineFail < orcaLines.size) {
      newVel = LinearProgram3(orcaLines, numObstLines, lineFail, runner.maxSpeed*0.01);
    }
    println(myVel, newVel, orcaLines, obstacleSegments)
    newVel *= 100;
    newVel = Vec2(newVel.x, -newVel.y);

    return runner.withNextStep(newVel * config.deltaTime, newVel);

  }

  def dot(v1: Vec2, v2: Vec2) : Double = v1.dot(v2)

  val RVO_EPSILON = 0.00001f;


  /**
   * <summary>Solves a one-dimensional linear program on a specified line
   * subject to linear constraints defined by lines and a circular
   * constraint.</summary>
   *
   * <returns>True if successful and the result of the linear program.</returns>
   *
   * <param name="lines">Lines defining the linear constraints.</param>
   * <param name="lineNo">The specified line constraint.</param>
   * <param name="radius">The radius of the circular constraint.</param>
   * <param name="optVelocity">The optimization velocity.</param>
   * <param name="directionOpt">True if the direction should be optimized.
   */

  def LinearProgram1(lines: List[Line], lineNo: Int, radius: Double, optVelocity: Vec2, directionOpt: Boolean): (Boolean, Vec2) = {
    var dotProduct = dot(lines(lineNo).point, lines(lineNo).direction);
    var discriminant = (dotProduct * dotProduct) + (radius * radius) - dot(lines(lineNo).point, lines(lineNo).point);

    if (discriminant < 0.0f) {
      /* Max speed circle fully invalidates line lineNo. */
      return (false, Vec2(0, 0));
    }

    val sqrtDiscriminant = math.sqrt(discriminant);
    var tLeft = -dotProduct - sqrtDiscriminant;
    var tRight = -dotProduct + sqrtDiscriminant;
    var result: Vec2 = Vec2(0, 0);
    var returnFalse = false
    for (i <- 0 until lineNo)
    {
      breakable
      {
        val denominator = determinant(Vec2x2(lines(lineNo).direction, lines(i).direction));
        val numerator = determinant(Vec2x2(lines(i).direction, lines(lineNo).point - lines(i).point));

        if (math.abs(denominator) <= RVO_EPSILON)
        {
          /* Lines lineNo and i are (almost) parallel. */
          if (numerator < 0.0f)
          {
            //return (false, Vec2(0, 0));
            returnFalse = true;
          }
          break();
        }

        val t = numerator / denominator;

        if (denominator >= 0.0f)
        {
          /* Line i bounds line lineNo on the right. */
          tRight = math.min(tRight, t);
        }
        else
        {
          /* Line i bounds line lineNo on the left. */
          tLeft = math.max(tLeft, t);
        }

        if (tLeft > tRight)
        {
          returnFalse = true;
          //return (false, Vec2(0, 0));
        }
      }
    }
    if (directionOpt)
    {
      /* Optimize direction. */
      if (dot(optVelocity, lines(lineNo).direction) > 0.0f)
      {
        /* Take right extreme. */
        result = lines(lineNo).point + lines(lineNo).direction * tRight;
      }
      else
      {
        /* Take left extreme. */
        result = lines(lineNo).point + lines(lineNo).direction * tLeft;
      }
    }
    else
    {
      /* Optimize closest point. */
      val t = dot(lines(lineNo).direction, (optVelocity - lines(lineNo).point));

      if (t < tLeft)
      {
        result = lines(lineNo).point + lines(lineNo).direction * tLeft;
      }
      else if (t > tRight)
      {
        result = lines(lineNo).point + lines(lineNo).direction * tRight;
      }
      else
      {
        result = lines(lineNo).point + lines(lineNo).direction * t;
      }
    }

    if (returnFalse)
      return (false, Vec2(0, 0))
    return (true, result);

  }

  /**
   * <summary>Solves a two-dimensional linear program subject to linear
   * constraints defined by lines and a circular constraint.</summary>
   *
   * <returns>The number of the line it fails on, and the number of lines
   * if successful and the result of the linear program.</returns>
   *
   * <param name="lines">Lines defining the linear constraints.</param>
   * <param name="radius">The radius of the circular constraint.</param>
   * <param name="optVelocity">The optimization velocity.</param>
   * <param name="directionOpt">True if the direction should be optimized.
   */
  def LinearProgram2(lines: List[Line], radius: Double, optVelocity: Vec2, directionOpt: Boolean): (Int, Vec2) = {
    var result: Vec2 = Vec2(0, 0)

    if (directionOpt) {
      /*
       * Optimize direction. Note that the optimization velocity is of
       * unit length in this case.
       */
      result = optVelocity * radius
    } else if (dot(optVelocity, optVelocity) > (radius * radius)) {
      /* Optimize closest point and outside circle. */
      result = optVelocity.normalized * radius
    } else {
      /* Optimize closest point and inside circle. */
      result = optVelocity
    }
    var endLoop = false;
    var it = lines.size;
    for (i <- lines.indices if !endLoop) {
      if (determinant(Vec2x2(lines(i).direction, lines(i).point - result)) > 0.0f) {
        /* Result does not satisfy constraint i. Compute new optimal result. */
        val (ret, newResult) = LinearProgram1(lines, i, radius, optVelocity, directionOpt)
        if (!ret)
          {
            endLoop = true;
            it = i;
          }
        else
          {
            result = newResult
          }
      }
    }
    return (it, result)
  }


  def LinearProgram3(lines: List[Line], numObstLines: Int, beginLine: Int, radius: Double): Vec2 = {
    var result: Vec2 = Vec2(0, 0);
    var distance = 0.0;

    for (i <- beginLine until lines.length)
    {
      if (determinant(Vec2x2(lines(i).direction, lines(i).point - result)) > distance) {
        /* Result does not satisfy constraint of line i. */
        val projLines = ListBuffer.empty[Line];

        for (ii <- 0 until numObstLines) {
          projLines += lines(ii)
        }

        for (j <- numObstLines until i) {
          breakable {
            var lineDir: Vec2 = Vec2(0, 0);
            var linePoint: Vec2 = Vec2(0, 0);

            val determinantVal = determinant(Vec2x2(lines(i).direction, lines(j).direction));

            if (math.abs(determinantVal) <= RVO_EPSILON) {
              /* Line i and line j are parallel. */
              if (dot(lines(i).direction, lines(j).direction) > 0.0f) {
                /* Line i and line j point in the same direction. */
                break();
              }
              else {
                /* Line i and line j point in opposite direction. */
                linePoint = (lines(i).point + lines(j).point) *  0.5f ;
              }
            }
            else {
              linePoint = lines(i).point +
                  lines(i).direction * (determinant(Vec2x2(lines(j).direction, lines(i).point - lines(j).point)) / determinantVal);

            }

            lineDir = (lines(j).direction - lines(i).direction).normalized;
            projLines.addOne(Line(lineDir, linePoint));
          }
        }

        val prevResult = result;
        val (id, newResult) = LinearProgram2(projLines.toList, radius, Vec2(-lines(i).direction.y, lines(i).direction.x), true);
        if (id < projLines.size)
          /*
           * This should in principle not happen. The result is by
           * definition already in the feasible region of this
           * linear program. If it fails, it is due to small
           * floating point error, and the current result is kept.
           */
          result = prevResult;
        else
          result = newResult;

        distance = determinant(Vec2x2(lines(i).direction, lines(i).point - result));
      }
    }
    return result;
  }

}
