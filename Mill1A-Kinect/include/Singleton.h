#include "stdafx.h"
#ifndef SINGLETON_H
#define SINGLETON_H

template <class T>
class Singleton
{
public:
    static T& Instance(void)
	{
		if (Singleton::_instance == 0)
		{
			Singleton::_instance = CreateInstance();
		}
		return *(Singleton::_instance);
	};
    static T* InstancePtr(void)
	{
		if (Singleton::_instance == 0)
		{
			Singleton::_instance = CreateInstance();
		}
		return Singleton::_instance;
	};
protected:
	virtual ~Singleton(){};
	Singleton(){};

private:
    static T* _instance;
    static T* CreateInstance()
	{
		return new T();
	};
};

template<class T>
T* Singleton<T>::_instance = 0;

#endif // SINGLETON_H