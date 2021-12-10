#include <nlohmann/json.hpp>
#include <iostream>

using json = nlohmann::json;

int main()
{
    json j2 = {
        {"pi", 3.141},
        {"happy", true},
        {"name", "Niels"},
        {"nothing", nullptr},
        {"answer", {{"everything", 42}}},
        {"list", {1, 0, 2}},
        {"object", {{"currency", "USD"}, {"value", 42.99}}}};
    std::cout << j2 << std::endl;   
}