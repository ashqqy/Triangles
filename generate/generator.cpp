#include <cassert>
#include <iostream>
#include <random>
#include <fstream>
#include <vector>

// GenerateRandomTriangles
int main ()
{
    std::size_t num = 0;
    std::size_t Dim = 0;
    std::cin >> num >> Dim;

    std::cout << num << "\n" << std::endl;

    std::mt19937 gen(42);
    std::uniform_real_distribution<double> main_dist(-100000, 100000);
    std::uniform_real_distribution<double> offset_dist(-1000, 1000);
    
    for (std::size_t i = 0; i < num; i++)
    {
        double x1 = main_dist(gen);
        double y1 = main_dist(gen);
        double z1 = 0.0;
        if (Dim == 3)
            z1 = main_dist(gen);
        
        double x2 = x1 + offset_dist(gen);
        double y2 = y1 + offset_dist(gen);
        double z2 = z1;
        if (Dim == 3)
            z2 = z1 + offset_dist(gen);
        
        double x3 = x1 + offset_dist(gen);
        double y3 = y1 + offset_dist(gen);
        double z3 = z1;
        if (Dim == 3)
            z3 = z1 + offset_dist(gen);
        
        if (Dim == 2)
        {
            std::cout << x1 << " " << y1 << std::endl;
            std::cout << x2 << " " << y2 << std::endl;
            std::cout << x3 << " " << y3 << std::endl;
        } 
        else
        { // Dim == 3
            std::cout << x1 << " " << y1 << " " << z1 << std::endl;
            std::cout << x2 << " " << y2 << " " << z2 << std::endl;
            std::cout << x3 << " " << y3 << " " << z3 << std::endl;
        }
        
        std::cout << std::endl;
    }
}
