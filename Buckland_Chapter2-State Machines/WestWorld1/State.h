/*
 * @Author: GuinGuinSzu guinguinboys@gmail.com
 * @Date: 2023-03-20 13:49:15
 * @LastEditors: GuinGuinSzu guinguinboys@gmail.com
 * @LastEditTime: 2023-03-20 14:29:14
 * @FilePath: \Programming-Game-AI-by-Example-src\Buckland_Chapter2-State Machines\WestWorld1\State.h
 * @Description: 
 * 
 * Copyright (c) 2023 by ${git_name_email}, All Rights Reserved. 
 */
#ifndef STATE_H
#define STATE_H
//------------------------------------------------------------------------
//
//  Name:   State.h
//
//  Desc:   abstract base class to define an interface for a state
//
//  Author: Mat Buckland 2002 (fup@ai-junkie.com)
//
//------------------------------------------------------------------------

// 抽象定义了

class Miner;

class State
{
public:

  virtual ~State(){}

  //this will execute when the state is entered
  virtual void Enter(Miner*)=0;

  //this is the state's normal update function
  virtual void Execute(Miner*)=0;

  //this will execute when the state is exited. (My word, isn't
  //life full of surprises... ;o))
  virtual void Exit(Miner*)=0;

};

#endif