#pragma once

#include <iostream>
#include <string>

class StringReturner
{
public:
    StringReturner() = default;

    const std::string getHello() const { return hello; };
    const std::string getWorld() const { return world; };

private:
    const std::string hello = "hello";
    const std::string world = "world";
};