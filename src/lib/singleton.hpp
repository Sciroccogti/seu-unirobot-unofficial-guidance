#pragma once

#include <memory>

template<class T>
class singleton
{
public:
    static std::shared_ptr<T> instance()
    {
        if (instance_ == nullptr)
        {
            instance_ = std::make_shared<T>();
        }

        return instance_;
    }
protected:
    singleton() {}
    T &operator=(const T &) {}
    static std::shared_ptr<T> instance_;
};

template<class T> std::shared_ptr<T> singleton<T>::instance_ = nullptr;
