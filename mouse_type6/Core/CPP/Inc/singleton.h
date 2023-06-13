/*
 * singleton.h
 *
 *  Created on: 2023/06/13
 *      Author: sato1
 */

#ifndef CPP_INC_SINGLETON_H_
#define CPP_INC_SINGLETON_H_


template <class T>
class Singleton {
public:
	static T& getInstance(void) {
		static T _instance;
		return _instance;
	}

protected:
	Singleton() = default;
	~Singleton() = default;

private:
	Singleton(const Singleton&) = delete;
	Singleton& operator=(const Singleton&) = delete;
	Singleton(Singleton&&) = delete;
	Singleton& operator=(Singleton&&) = delete;
};


#endif /* CPP_INC_SINGLETON_H_ */
