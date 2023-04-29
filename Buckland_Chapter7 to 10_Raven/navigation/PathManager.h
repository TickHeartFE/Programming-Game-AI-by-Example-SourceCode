#ifndef PATH_MANAGER_H
#define PATH_MANAGER_H
#pragma warning (disable:4786)
//-----------------------------------------------------------------------------
//
//  Name:   PathManager.h
//
//  Author: Mat Buckland (www.ai-junkie.com)
//
//  Desc:   a template class to manage a number of graph searches, and to 
//          distribute the calculation of each search over several update-steps
//-----------------------------------------------------------------------------
#include <list>
#include <cassert>



template <class path_planner>
class PathManager
{
private:

  //a container of all the active search requests
  std::list<path_planner*>  m_SearchRequests;

  //this is the total number of search cycles allocated to the manager. 
  //Each update-step these are divided equally amongst all registered path
  //requests
  unsigned int              m_iNumSearchCyclesPerUpdate;

public:
    
  PathManager(unsigned int NumCyclesPerUpdate):m_iNumSearchCyclesPerUpdate(NumCyclesPerUpdate){}

  //every time this is called the total amount of search cycles available will
  //be shared out equally between all the active path requests. If a search
  //completes successfully or fails the method will notify the relevant bot
  void UpdateSearches();

  //a path planner should call this method to register a search with the 
  //manager. (The method checks to ensure the path planner is only registered
  //once)
  void Register(path_planner* pPathPlanner);

  void UnRegister(path_planner* pPathPlanner);

  //returns the amount of path requests currently active.
  int  GetNumActiveSearches()const{return m_SearchRequests.size();}
};

///////////////////////////////////////////////////////////////////////////////
//------------------------- UpdateSearches ------------------------------------
//
//  This method iterates through all the active path planning requests 
//  updating their searches until the user specified total number of search
//  cycles has been satisfied.
//
//  If a path is found or the search is unsuccessful the relevant agent is
//  notified accordingly by Telegram
//  在这里进行Update的操作, 迭代所有活动路径规划请求并同时更新它们的搜索, 直到满足用户指定的搜索周期总数, 如果找到路径或搜索不成功, 则Telegram会通知对应的代理
//-----------------------------------------------------------------------------
template <class path_planner>
inline void PathManager<path_planner>::UpdateSearches()
{
  // get NumCylesRemaining记录最大的更新次数
  int NumCyclesRemaining = m_iNumSearchCyclesPerUpdate;

  //iterate through the search requests until either all requests have been
  //fulfilled or there are no search cycles remaining for this update-step.
  std::list<path_planner*>::iterator curPath = m_SearchRequests.begin();
  while (NumCyclesRemaining-- && !m_SearchRequests.empty())
  {
    // make one search cycle of this path request
    // 执行一次Cycle搜索, 而不是一次全部执行完路径
    int result = (*curPath)->CycleOnce();

    //if the search has terminated remove from the list
    if ( (result == target_found) || (result == target_not_found) )
    {
      //remove this path from the path list
      curPath = m_SearchRequests.erase(curPath);       
    }
    //move on to the next
    else
    {
      ++curPath;
    }

    // the iterator may now be pointing to the end of the list. If this is so,
    // it must be reset to the beginning.
    // 这里进行开头重置
    // it must be reset to the beginning
    if(curPath == m_SearchRequests.end())
    {
      curPath = m_SearchRequests.begin();
    }

  } // 这里退出搜索, quit the search and end while
}

//--------------------------- Register ----------------------------------------
//
//  this is called to register a search with the manager.
//-----------------------------------------------------------------------------
template <class path_planner>
inline void PathManager<path_planner>::Register(path_planner* pPathPlanner)
{
  //make sure the bot does not already have a current search in the queue
  if(std::find(m_SearchRequests.begin(),
               m_SearchRequests.end(),
               pPathPlanner) == m_SearchRequests.end())
  {  
    // add to the list
    // 注册相当于把pPathPlanner给压入到list中即可
    m_SearchRequests.push_back(pPathPlanner);
  }
}

//----------------------------- UnRegister ------------------------------------
//-----------------------------------------------------------------------------
template <class path_planner>
inline void PathManager<path_planner>::UnRegister(path_planner* pPathPlanner)
{
  m_SearchRequests.remove(pPathPlanner);
}





#endif