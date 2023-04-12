#ifndef SOCCERBALL_H
#define SOCCERBALL_H
#pragma warning (disable:4786)
//------------------------------------------------------------------------
//
//  Name: SoccerBall.h
//
//  Desc: Class to implement a soccer ball. This class inherits from
//        MovingEntity and provides further functionality for collision
//        testing and position prediction.
//
//  Author: Mat Buckland 2003 (fup@ai-junkie.com)
//
//------------------------------------------------------------------------
#include <vector>

#include "Game/MovingEntity.h"
#include "constants.h"


class Wall2D;
class PlayerBase;


class SoccerBall : public MovingEntity
{
private:

  //keeps a record of the ball's position at the last update
  Vector2D                  m_vOldPos;

  //a local reference to the Walls that make up the pitch boundary
  const std::vector<Wall2D>& m_PitchBoundary;                                      



public:
  //tests to see if the ball has collided with a ball and reflects 
  //the ball's velocity accordingly

  // 测试球是否和墙碰撞，相应地反射球的速度
  void TestCollisionWithWalls(const std::vector<Wall2D>& walls);

  SoccerBall(Vector2D           pos,            
             double               BallSize,
             double               mass,
             std::vector<Wall2D>& PitchBoundary):
  
      //set up the base class
      MovingEntity(pos, // position
                  BallSize, // Radius
                  Vector2D(0,0), // velocity
                  -1.0,                //max speed - unused
                  Vector2D(0,1), // heading
                  mass, // mass
                  Vector2D(1.0,1.0),  //scale     - unused
                  0,                   //turn rate - unused
                  0),                  //max force - unused
     m_PitchBoundary(PitchBoundary)
  {}
  
  //implement base class Update
  void      Update();

  //implement base class Render
  void      Render();

  //a soccer ball doesn't need to handle messages
  bool      HandleMessage(const Telegram& msg){return false;}

  //this method applies a directional force to the ball (kicks it!)
  void      Kick(Vector2D direction, double force);

  //given a kicking force and a distance to traverse defined by start
  //and finish points, this method calculates how long it will take the
  //ball to cover the distance.

  // 给定一个kicking force, 并用一个start 和 finish来定义一个移动距离
  // 返回球经过这段距离要花多久
  double    TimeToCoverDistance(Vector2D from,
                               Vector2D to,
                               double     force)const;

  //this method calculates where the ball will in 'time' seconds
  Vector2D FuturePosition(double time)const;

  //this is used by players and goalkeepers to 'trap' a ball -- to stop
  //it dead. That player is then assumed to be in possession of the ball
  //and m_pOwner is adjusted accordingly
  void      Trap(){m_vVelocity.Zero();}  

  Vector2D  OldPos()const{return m_vOldPos;}
  
  //this places the ball at the desired location and sets its velocity to zero
  void      PlaceAtPosition(Vector2D NewPos);
};



//this can be used to vary the accuracy of a player's kick.
Vector2D AddNoiseToKick(Vector2D BallPos, Vector2D BallTarget);



#endif