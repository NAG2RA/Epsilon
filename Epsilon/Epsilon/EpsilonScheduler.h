#pragma once
#include<vector>
#include<iostream>
#include<functional>
class EpsilonScheduler
{

    std::vector<std::function<void()>> taskQueue;
public:
    template<typename Func, typename... Args>
    void Schedule(Func&& f, Args&&... args) {
        taskQueue.emplace_back(
            [f = std::forward<Func>(f),
            // Note: We capture the arguments into a tuple here
            args = std::make_tuple(std::forward<Args>(args)...)]() mutable {

                std::apply(f, args);
            });
    }

    void Run();
};

