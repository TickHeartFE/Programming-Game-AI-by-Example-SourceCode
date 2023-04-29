#include "Raven_SensoryMemory.h"
#include "Raven_Game.h"
#include "time/crudetimer.h"
#include "misc/cgdi.h"
#include "misc/Stream_Utility_Functions.h"

//------------------------------- ctor ----------------------------------------
//-----------------------------------------------------------------------------
 Raven_SensoryMemory:: Raven_SensoryMemory(Raven_Bot* owner,
                                           double MemorySpan):m_pOwner(owner),
                                                            m_dMemorySpan(MemorySpan)
                                                          
{}

//--------------------- MakeNewRecordIfNotAlreadyPresent ----------------------

void Raven_SensoryMemory::MakeNewRecordIfNotAlreadyPresent(Raven_Bot* pOpponent)
{
  //else check to see if this Opponent already exists in the memory. If it doesn't,
  //create a new record
  if (m_MemoryMap.find(pOpponent) == m_MemoryMap.end())
  {
    m_MemoryMap[pOpponent] = MemoryRecord();
  }
}

//------------------------ RemoveBotFromMemory --------------------------------
//
//  this removes a bot's record from memory
//-----------------------------------------------------------------------------
void Raven_SensoryMemory::RemoveBotFromMemory(Raven_Bot* pBot)
{
  MemoryMap::iterator record = m_MemoryMap.find(pBot);
  
  if (record != m_MemoryMap.end())
  {
    m_MemoryMap.erase(record);
  }
}
  
//----------------------- UpdateWithSoundSource -------------------------------
//
// this updates the record for an individual opponent. Note, there is no need to
// test if the opponent is within the FOV because that test will be done when the
// UpdateVision method is called
// 对于角色声源的MemoryMap的更新，不要考虑FOV
// 只需要对视觉的更新考虑AI的FOV
//-----------------------------------------------------------------------------
void Raven_SensoryMemory::UpdateWithSoundSource(Raven_Bot* pNoiseMaker)
{
  //make sure the bot being examined is not this bot
  if (m_pOwner != pNoiseMaker)
  {
    // if the bot is already part of the memory then update its data, else
    // create a new memory record and add it to the memory
    MakeNewRecordIfNotAlreadyPresent(pNoiseMaker);

    MemoryRecord& info = m_MemoryMap[pNoiseMaker];

    // test if there is LOS between bots 
    // 测试两者之间没有墙壁的阻挡
    // 注意到墙壁是可以阻挡视线和听觉
    // 这里进行一个墙壁判定
    if(m_pOwner->GetWorld()->isLOSOkay(m_pOwner->Pos(), pNoiseMaker->Pos()))
    {
      // info.bShootable = true;
      // 记录bot的上一个位置
      // info.vLastSensedPosition = pNoiseMaker->Pos();

      
      info.bShootable = true;
      // 记录下sense的位置
      info.vLastSensedPosition = pNoiseMaker->Pos();
    }
    else
    {
      // 被墙壁阻挡
      // info.bShootable = false;
      info.bShootable = false;
    }
    
    // record the time it was sensed
    info.fTimeLastSensed = (double)Clock->GetCurrentTime();
  }
}

//----------------------------- UpdateVision ----------------------------------
//
//  this method iterates through all the bots in the game world to test if
//  they are in the field of view. Each bot's memory record is updated
//  accordingly
//-----------------------------------------------------------------------------
void Raven_SensoryMemory::UpdateVision()
{
  // for each bot in the world test to see if it is visible to the owner of
  // this class
  // 首先获取所有的Bots
  const std::list<Raven_Bot*>& bots = m_pOwner->GetWorld()->GetAllBots();

  // 然后在这里遍历所有Bots
  std::list<Raven_Bot*>::const_iterator curBot;
  for (curBot = bots.begin(); curBot!=bots.end(); ++curBot)
  {
    // make sure the bot being examined is not this bot
    if(m_pOwner != *curBot)
    {
      // make sure it is part of the memory map
      // 保证存在记忆图 
      MakeNewRecordIfNotAlreadyPresent(*curBot);

      //get a reference to this bot's data
      MemoryRecord& info = m_MemoryMap[*curBot];

      // test if there is LOS between bots 
      // 后面更新info的相关消息
      if(m_pOwner->GetWorld()->isLOSOkay(m_pOwner->Pos(), (*curBot)->Pos()))
      {
        info.bShootable = true;

        //test if the bot is within FOV
        if (isSecondInFOVOfFirst(m_pOwner->Pos(),
                                 m_pOwner->Facing(),
                                 (*curBot)->Pos(),
                                  m_pOwner->FieldOfView()))
        {
          // 记录对应的感知时间
          info.fTimeLastSensed = Clock->GetCurrentTime();
          // 记录对应的感知位置
          info.vLastSensedPosition = (*curBot)->Pos();
          // 记录对应的看到的时间
          info.fTimeLastVisible = Clock->GetCurrentTime();

          // // 维护bWithinFOV变量
          // if(info.bWithinFOV == false)
          // {
          //   info.bWithinFOV = true;
          //   // 记录下首次感知的时间
          //   info.fTimeBecameVisible = info.fTimeLastSensed;
          // }

          // 相当于从不可见->可见
          if(info.bWithinFOV == false) {
            // info.bWithinFOV = true;
            // info.fTimeBecameVisible = info.fTimeLastSensed;
            info.bWithinFOV = true;
            // 此时记录下从不可见到可见的瞬间时间, 作为BecameVisible
            info.fTimeBecameVisible = info.fTimeLastSensed;
          }

        }
        else
        {
          info.bWithinFOV = false;         
        }
      }
      else
      {
        info.bShootable = false;
        info.bWithinFOV = false;
      }
    }
  }//next bot
}


//------------------------ GetListOfRecentlySensedOpponents -------------------
//
//  returns a list of the bots that have been sensed recently
//-----------------------------------------------------------------------------
std::list<Raven_Bot*> 
Raven_SensoryMemory::GetListOfRecentlySensedOpponents()const
{
  //this will store all the opponents the bot can remember
  std::list<Raven_Bot*> opponents;

  double CurrentTime = Clock->GetCurrentTime();

  MemoryMap::const_iterator curRecord = m_MemoryMap.begin();
  for (curRecord; curRecord!=m_MemoryMap.end(); ++curRecord)
  {
    //if this bot has been updated in the memory recently, add to list
    if ( (CurrentTime - curRecord->second.fTimeLastSensed) <= m_dMemorySpan)
    {
      opponents.push_back(curRecord->first);
    }
  }

  return opponents;
}

//----------------------------- isOpponentShootable --------------------------------
//
//  returns true if the bot given as a parameter can be shot (ie. its not
//  obscured by walls)
//  这里进行查表，然后返回是否可以Shootable
//-----------------------------------------------------------------------------
bool Raven_SensoryMemory::isOpponentShootable(Raven_Bot* pOpponent)const
{
  MemoryMap::const_iterator it = m_MemoryMap.find(pOpponent);
 
  if (it != m_MemoryMap.end())
  {
    return it->second.bShootable;
  }

  return false;
}

//----------------------------- isOpponentWithinFOV --------------------------------
//
//  returns true if the bot given as a parameter is within FOV
//-----------------------------------------------------------------------------
bool  Raven_SensoryMemory::isOpponentWithinFOV(Raven_Bot* pOpponent)const
{
  MemoryMap::const_iterator it = m_MemoryMap.find(pOpponent);
 
  // if (it != m_MemoryMap.end())
  // {
  //   return it->second.bWithinFOV;
  // }

  if(it != m_MemoryMap.end()) {
    return it->second.bWithinFOV;
  }

  return false;
}

//---------------------------- GetLastRecordedPositionOfOpponent -------------------
//
//  returns the last recorded position of the bot
//-----------------------------------------------------------------------------
Vector2D  Raven_SensoryMemory::GetLastRecordedPositionOfOpponent(Raven_Bot* pOpponent)const
{
  MemoryMap::const_iterator it = m_MemoryMap.find(pOpponent);
 
  if (it != m_MemoryMap.end())
  {
    return it->second.vLastSensedPosition;
  }

  throw std::runtime_error("< Raven_SensoryMemory::GetLastRecordedPositionOfOpponent>: Attempting to get position of unrecorded bot");
}

//----------------------------- GetTimeOpponentHasBeenVisible ----------------------
//
//  returns the amount of time the given bot has been visible
//-----------------------------------------------------------------------------
double  Raven_SensoryMemory::GetTimeOpponentHasBeenVisible(Raven_Bot* pOpponent)const
{
  MemoryMap::const_iterator it = m_MemoryMap.find(pOpponent);
 
  if (it != m_MemoryMap.end() && it->second.bWithinFOV)
  {
    return Clock->GetCurrentTime() - it->second.fTimeBecameVisible;
  }

  return 0;
}

//-------------------- GetTimeOpponentHasBeenOutOfView ------------------------
//
//  returns the amount of time the given opponent has remained out of view
//  returns a high value if opponent has never been seen or not present
//-----------------------------------------------------------------------------
double Raven_SensoryMemory::GetTimeOpponentHasBeenOutOfView(Raven_Bot* pOpponent)const
{
  MemoryMap::const_iterator it = m_MemoryMap.find(pOpponent);
 
  if (it != m_MemoryMap.end())
  {
    // 现在的时间 - fTimeLastVisible
    // 这里计算出在视野外的时间
    return Clock->GetCurrentTime() - it->second.fTimeLastVisible;
  }

  return MaxDouble;
}

//------------------------ GetTimeSinceLastSensed ----------------------
//
//  returns the amount of time the given bot has been visible
//  返回bot被看到的时长
//-----------------------------------------------------------------------------
double  Raven_SensoryMemory::GetTimeSinceLastSensed(Raven_Bot* pOpponent)const
{
  // MemoryMap::const_iterator it = m_MemoryMap.find(pOpponent);
 
  // if (it != m_MemoryMap.end() && it->second.bWithinFOV)
  // {
  //   return Clock->GetCurrentTime() - it->second.fTimeLastSensed;
  // }
  // return 0;

  MemoryMap::const_iterator it = m_MemoryMap.find(pOpponent);
  if(it != m_MemoryMap.end() && it->second.bWithinFOV)
  {
    return Clock->GetCurrentTime() - it->second.fTimeLastSensed;
  }

  return 0;

}

//---------------------- RenderBoxesAroundRecentlySensed ----------------------
//
//  renders boxes around the opponents it has sensed recently.
//-----------------------------------------------------------------------------
void  Raven_SensoryMemory::RenderBoxesAroundRecentlySensed()const
{
  std::list<Raven_Bot*> opponents = GetListOfRecentlySensedOpponents();
  std::list<Raven_Bot*>::const_iterator it;
  for (it = opponents.begin(); it != opponents.end(); ++it)
  {
    gdi->OrangePen();
    Vector2D p = (*it)->Pos();
    double   b = (*it)->BRadius();
      
    gdi->Line(p.x-b, p.y-b, p.x+b, p.y-b);
    gdi->Line(p.x+b, p.y-b, p.x+b, p.y+b);
    gdi->Line(p.x+b, p.y+b, p.x-b, p.y+b);
    gdi->Line(p.x-b, p.y+b, p.x-b, p.y-b);
  }

}