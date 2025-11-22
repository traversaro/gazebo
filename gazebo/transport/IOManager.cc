/*
 * Copyright (C) 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#include <atomic>
#include <boost/bind/bind.hpp>
#include <boost/thread/thread.hpp>
#include <iostream>
#include <boost/version.hpp>

#if BOOST_VERSION >= 108800
#include <boost/asio/executor_work_guard.hpp>
#endif

#include "gazebo/transport/IOManager.hh"

namespace gazebo
{
namespace transport
{
/////////////////////////////////////////////////
class IOManagerPrivate
{
  /// \brief IO context/service.
  public: AsioIO *io = nullptr;

#if BOOST_VERSION >= 108800
  /// \brief Work guard keeps the io_context running.
  public: boost::asio::executor_work_guard<AsioIO::executor_type> *work =
      nullptr;
#else
  /// \brief Use io_service::work to keep the io_service running in thread.
  public: boost::asio::io_service::work *work = nullptr;
#endif

  /// \brief Reference count of connections using this IOManager.
  public: std::atomic_int count;

  /// \brief Thread for IOManager.
  public: boost::thread *thread = nullptr;
};

/////////////////////////////////////////////////
IOManager::IOManager()
  : dataPtr(new IOManagerPrivate)
{
  this->dataPtr->io = new AsioIO;
#if BOOST_VERSION >= 108800
  this->dataPtr->work =
      new boost::asio::executor_work_guard<AsioIO::executor_type>(
      this->dataPtr->io->get_executor());
#else
  this->dataPtr->work = new boost::asio::io_service::work(
      *this->dataPtr->io);
#endif
  this->dataPtr->count = 0;
  this->dataPtr->thread = new boost::thread(boost::bind(
      &AsioIO::run, this->dataPtr->io));
}

/////////////////////////////////////////////////
IOManager::~IOManager()
{
  this->Stop();

  delete this->dataPtr->work;
  this->dataPtr->work = nullptr;

  delete this->dataPtr->io;
  this->dataPtr->io = nullptr;

  delete this->dataPtr;
  this->dataPtr = nullptr;
}

/////////////////////////////////////////////////
void IOManager::Stop()
{
#if BOOST_VERSION >= 108800
  this->dataPtr->io->restart();
#else
  this->dataPtr->io->reset();
#endif
  this->dataPtr->io->stop();
  if (this->dataPtr->thread)
  {
    this->dataPtr->thread->join();
    delete this->dataPtr->thread;
    this->dataPtr->thread = nullptr;
  }
}

/////////////////////////////////////////////////
AsioIO &IOManager::GetIO()
{
  return *this->dataPtr->io;
}

/////////////////////////////////////////////////
void IOManager::IncCount()
{
  this->dataPtr->count++;
}

/////////////////////////////////////////////////
void IOManager::DecCount()
{
  this->dataPtr->count--;
}

/////////////////////////////////////////////////
unsigned int IOManager::GetCount() const
{
  return this->dataPtr->count;
}
}
}
