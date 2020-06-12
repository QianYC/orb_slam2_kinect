//
// Created by yc_qian on 19-12-20.
//

#ifndef ORB_SLAM2_KINETIC_OPENMP_MUTEX_H
#define ORB_SLAM2_KINETIC_OPENMP_MUTEX_H

#include "../../config.h"

#ifdef G2O_OPENMP
#include <omp.h>
#else
#include <cassert>
#endif

namespace g2o {

#ifdef G2O_OPENMP

    /**
   * \brief Mutex realized via OpenMP
   */
  class OpenMPMutex
  {
    public:
      OpenMPMutex() { omp_init_lock(&_lock); }
      ~OpenMPMutex() { omp_destroy_lock(&_lock); }
      void lock() { omp_set_lock(&_lock); }
      void unlock() { omp_unset_lock(&_lock); }
    protected:
      omp_lock_t _lock;
  };

#else

    /*
     * dummy which does nothing in case we don't have OpenMP support.
     * In debug mode, the mutex allows to verify the correct lock and unlock behavior
     */
    class OpenMPMutex
    {
    public:
#ifdef NDEBUG
        OpenMPMutex() {}
#else
        OpenMPMutex() : _cnt(0) {}
#endif
        ~OpenMPMutex() { assert(_cnt == 0 && "Freeing locked mutex");}
        void lock() { assert(++_cnt == 1 && "Locking already locked mutex");}
        void unlock() { assert(--_cnt == 0 && "Trying to unlock a mutex which is not locked");}
    protected:
#ifndef NDEBUG
        char _cnt;
#endif
    };

#endif

    /**
     * \brief lock a mutex within a scope
     */
    class ScopedOpenMPMutex
    {
    public:
        explicit ScopedOpenMPMutex(OpenMPMutex* mutex) : _mutex(mutex) { _mutex->lock(); }
        ~ScopedOpenMPMutex() { _mutex->unlock(); }
    private:
        OpenMPMutex* const _mutex;
        ScopedOpenMPMutex(const ScopedOpenMPMutex&);
        void operator=(const ScopedOpenMPMutex&);
    };

}

#endif //ORB_SLAM2_KINETIC_OPENMP_MUTEX_H
