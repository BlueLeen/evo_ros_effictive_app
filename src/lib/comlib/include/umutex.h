#ifndef UMUTEX_H
#define UMUTEX_H

#include <pthread.h>

class UMutex
{
public:
	UMutex()
	{
		pthread_mutexattr_t attr;
		pthread_mutexattr_init(&attr);
		pthread_mutexattr_settype(&attr,PTHREAD_MUTEX_RECURSIVE);
		pthread_mutex_init(&M,&attr);
		pthread_mutexattr_destroy(&attr);
	}

	virtual ~UMutex()
	{
		pthread_mutex_unlock(&M); pthread_mutex_destroy(&M);
	}

	/**
	 * Lock the mutex.
	 */
	int lock() const
	{
		return pthread_mutex_lock(&M);
	}

	int lockTry() const
	{
		return pthread_mutex_trylock(&M);
	}

	/**
	 * Unlock the mutex.
	 */
	int unlock() const
	{
		return pthread_mutex_unlock(&M);
	}

private:
	mutable pthread_mutex_t M;
	void operator=(UMutex &M) {}
	UMutex( const UMutex &M ) {}
};

class UScopeMutex
{
public:
	UScopeMutex(const UMutex & mutex) :
		mutex_(mutex)
	{
		mutex_.lock();
	}
	// backward compatibility
	UScopeMutex(UMutex * mutex) :
		mutex_(*mutex)
	{
		mutex_.lock();
	}
	~UScopeMutex()
	{
		mutex_.unlock();
	}
private:
	const UMutex & mutex_;
};


#endif
