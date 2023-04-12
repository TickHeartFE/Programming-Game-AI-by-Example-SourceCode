#include "GoalKeeper.h"
#include "misc/Cgdi.h"
#include "SteeringBehaviors.h"
#include "SoccerTeam.h"
#include "SoccerPitch.h"
#include "2D/transformations.h"
#include "GoalKeeperStates.h"
#include "Goal.h"
#include "game/EntityFunctionTemplates.h"
#include "ParamLoader.h"



//----------------------------- ctor ------------------------------------
//-----------------------------------------------------------------------
GoalKeeper::GoalKeeper(SoccerTeam* home_team,
                       int                home_region,
                       State<GoalKeeper>* start_state,
                       Vector2D           heading,
                       Vector2D           velocity,
                       double              mass,
                       double              max_force,
                       double              max_speed,
                       double              max_turn_rate,
                       double              scale): PlayerBase(home_team,
                                                             home_region,
                                                             heading,
                                                             velocity,
                                                             mass,
                                                             max_force,
                                                             max_speed,
                                                             max_turn_rate,
                                                             scale,
                                                             PlayerBase::goal_keeper)


{
  //set up the state machine
  m_pStateMachine = new StateMachine<GoalKeeper>(this);

  m_pStateMachine->SetCurrentState(start_state);
  m_pStateMachine->SetPreviousState(start_state);
  m_pStateMachine->SetGlobalState(GlobalKeeperState::Instance());

  m_pStateMachine->CurrentState()->Enter(this);
}



//-------------------------- Update --------------------------------------

void GoalKeeper::Update() {
  // run the logic for the current state
  // fist upate the StateMachine 
  m_pStateMachine->Update();

  // calculate the combined force from each steering behavior 
  // then calculate the combined force from each steering behaviors
  Vector2D SteeringForce = m_pSteering->Calculate();



  //Acceleration = Force/Mass
  Vector2D Acceleration = SteeringForce / m_dMass;

  //update velocity
  m_vVelocity += Acceleration;

  //make sure player does not exceed maximum velocity
  m_vVelocity.Truncate(m_dMaxSpeed);

  //update the position
  m_vPosition += m_vVelocity;


  //enforce a non-penetration constraint if desired
  if(Prm.bNonPenetrationConstraint) {
    EnforceNonPenetrationContraint(this, AutoList<PlayerBase>::GetAllMembers());
  }

  //update the heading if the player has a non zero velocity
  if(!m_vVelocity.isZero()) {
    m_vHeading = Vec2DNormalize(m_vVelocity);

    m_vSide = m_vHeading.Perp();
  }

  //look-at vector always points toward the ball
  if(!Pitch()->GoalKeeperHasBall()) {
    m_vLookAt = Vec2DNormalize(Ball()->Pos() - Pos());
  }
}


bool GoalKeeper::BallWithinRangeForIntercept()const {
  return (Vec2DDistanceSq(Team()->HomeGoal()->Center(), Ball()->Pos()) <=
          Prm.GoalKeeperInterceptRangeSq);
}

bool GoalKeeper::TooFarFromGoalMouth()const {
  return (Vec2DDistanceSq(Pos(), GetRearInterposeTarget()) >
          Prm.GoalKeeperInterceptRangeSq);
}

Vector2D GoalKeeper::GetRearInterposeTarget()const {
  double xPosTarget = Team()->HomeGoal()->Center().x;

  // 感觉Ball.pos.y来动态调整插入的后方目标, 当球上半场的时候, 
  // 那么球的的位置也同理移动到上半场, 反之移动到下半场
  // prm.goalWidth是用来调整的幅度大小
  double yPosTarget = Pitch()->PlayingArea()->Center().y +
  Prm.GoalWidth * (Ball()->Pos().y /
  Pitch()->PlayingArea()->Height() - 0.5);

  return Vector2D(xPosTarget, yPosTarget);
}

//-------------------- HandleMessage -------------------------------------
//
//  routes any messages appropriately
//------------------------------------------------------------------------
bool GoalKeeper::HandleMessage(const Telegram& msg) {
  return m_pStateMachine->HandleMessage(msg);
}

//--------------------------- Render -------------------------------------
//
//------------------------------------------------------------------------
void GoalKeeper::Render() {
  if(Team()->Color() == SoccerTeam::blue)
    gdi->BluePen();
  else
    gdi->RedPen();

  m_vecPlayerVBTrans = WorldTransform(m_vecPlayerVB,
                                       Pos(),
                                       m_vLookAt,
                                       m_vLookAt.Perp(),
                                       Scale());

  gdi->ClosedShape(m_vecPlayerVBTrans);

  //draw the head
  gdi->BrownBrush();
  gdi->Circle(Pos(), 6);

  //draw the ID
  if(Prm.bIDs) {
    gdi->TextColor(0, 170, 0);;
    gdi->TextAtPos(Pos().x - 20, Pos().y - 20, ttos(ID()));
  }

  //draw the state
  if(Prm.bStates) {
    gdi->TextColor(0, 170, 0);
    gdi->TransparentText();
    gdi->TextAtPos(m_vPosition.x, m_vPosition.y - 20, std::string(m_pStateMachine->GetNameOfCurrentState()));
  }
}