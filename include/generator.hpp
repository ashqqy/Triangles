#include <iostream>
#include <fstream>
#include <random>
#include <string>

namespace TrianglesGenerating
{

std::string GenerateRandomTriangles(std::size_t triangles_count, std::size_t Dim)
{
    assert (Dim == 2 || Dim == 3);
    std::string filename = "triangles_" + std::to_string(Dim) + "D_" + 
                          std::to_string(triangles_count) + ".dat";
    
    std::ofstream outfile(filename);
    
    outfile << triangles_count << "\n" << std::endl;

    std::mt19937 gen(42);
    std::uniform_real_distribution<double> main_dist(-100000, 100000);
    std::uniform_real_distribution<double> offset_dist;
    if (Dim == 3)
        offset_dist = std::uniform_real_distribution<double>(-1000, 1000);
    if (Dim == 2)
        offset_dist = std::uniform_real_distribution<double>(-100, 100);
    
    for (std::size_t i = 0; i < triangles_count; i++)
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
            outfile << x1 << " " << y1 << std::endl;
            outfile << x2 << " " << y2 << std::endl;
            outfile << x3 << " " << y3 << std::endl;
        } 
        else
        { // Dim == 3
            outfile << x1 << " " << y1 << " " << z1 << std::endl;
            outfile << x2 << " " << y2 << " " << z2 << std::endl;
            outfile << x3 << " " << y3 << " " << z3 << std::endl;
        }
        
        outfile << std::endl;
    }
    
    outfile.close();
    return filename;
}

} // namespace TrianglesGenerating
