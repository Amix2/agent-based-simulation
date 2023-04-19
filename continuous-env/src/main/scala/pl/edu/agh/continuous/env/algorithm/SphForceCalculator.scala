package pl.edu.agh.continuous.env.algorithm

import pl.edu.agh.continuous.env.common.CellBoundedPosition.PositionExtensions
import pl.edu.agh.continuous.env.common.CollisionAvoidance.RunnerCollisionAvoidanceExtensions
import pl.edu.agh.xinuk.algorithm.MathUtils.DoubleExtensions
import pl.edu.agh.continuous.env.common.ObstacleMapping
import pl.edu.agh.continuous.env.common.RunnerPhysics.RunnerExtensions
import pl.edu.agh.continuous.env.common.ToVec2Conversions.SignalMapConversionExtensions
import pl.edu.agh.continuous.env.common.geometry.Algorithms.{LineIntersection, intersect}
import pl.edu.agh.continuous.env.common.geometry.Line
import pl.edu.agh.continuous.env.config.ContinuousEnvConfig
import pl.edu.agh.continuous.env.model.{ContinuousEnvCell, MoveCompletion, Runner, RunnerOccupied}
import pl.edu.agh.xinuk.algorithm.{Plan, PlanCreator, Plans, StateUpdate, Vec2}
import pl.edu.agh.xinuk.model._
import pl.edu.agh.xinuk.model.continuous._
import pl.edu.agh.xinuk.model.grid.GridDirection
import pl.edu.agh.xinuk.model.grid.GridDirection.{BottomLeft, BottomRight, TopLeft, TopRight}

import java.util.{NoSuchElementException, UUID}
import scala.collection.mutable.ListBuffer

object SphForceCalculator {
  def adjustSphForRunner(runner: Runner,
                         signalMap: SignalMap,
                         cell: ContinuousEnvCell,
                         neighbourContents: Map[(ContinuousEnvCell, UUID), Direction],
                         config: ContinuousEnvConfig): Runner = {

    val (agentMessages, _) = signalMap.toObjectMessagesSplit match {
      case (agents, obstacles) => (agents, obstacles)
      case _ => (List.empty[AgentMessage], List.empty[ObstacleMessage])
    }

    var myDensity: Double = 0;
    val myPos = runner.globalCellPosition(config) + cell.BaseCoordinates(config);
    val myVel = runner.velocity;

    agentMessages.foreach((msg =>
      myDensity += msg.mass * kernel((myPos - msg.pos).length, config.sphConfig.kernelSize*100)))

    var myPressure: Double = 0;
    if (myDensity > config.sphConfig.targetDensity)
      myPressure = config.sphConfig.stiffness * (myDensity - config.sphConfig.targetDensity);

    var myForceViscosity: Vec2 = Vec2(0, 0);
    var myForcePressure: Vec2 = Vec2(0, 0);

    agentMessages.foreach({ msg =>
      if(msg.sphData.density > 0) {
        val d2K = kernelSecondDerivative((myPos - msg.pos).length, config.sphConfig.kernelSize*100);
        if (d2K > 0) {
          myForceViscosity += (myVel - msg.vel) * config.sphConfig.viscosity / (msg.sphData.density * myDensity) * d2K;
          val dir = myPos - msg.pos
          myForcePressure += dir *
            (myPressure / (myDensity * myDensity) + msg.sphData.pressure / (msg.sphData.density * msg.sphData.density)) *
            kernelDerivative((myPos - msg.pos).length, config.sphConfig.kernelSize*100);
        }
      }
    })
    val force = myForceViscosity - myForcePressure;
    //println(force)
    return runner.withNewSphData(SphObjectData(myPressure, myDensity)).withIncreasedForce(Vec2(force.x, -force.y))
  }

  def kernel(x: Double, h: Double): Double = {
    var x_scaled = x / h
    if (x_scaled >= 1) return 0.0f
    return 315 * math.pow(1 - math.pow(x_scaled, 2), 3) / (64 * math.Pi)
  }

  def kernelDerivative(x: Double, h: Double): Double = {
    var x_scaled = x / h
    if (x_scaled >= 1) return 0.0f
    return -45 * math.pow(1 - x_scaled, 2) / math.Pi.toFloat
  }

  def kernelSecondDerivative(x: Double, h: Double): Double = {
    var x_scaled = x / h
    if (x_scaled >= 1) return 0.0f
    return 45 * (1 - x_scaled) / math.Pi
  }
}
