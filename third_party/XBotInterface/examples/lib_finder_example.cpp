#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <cstdlib>
#include <iostream>
#include <iomanip>

int main(int argc, char **argv)
{
 
    char * ld_library_path_char = std::getenv("LD_LIBRARY_PATH");
    std::string ld_library_path(ld_library_path_char);
    
    std::cout << "LD_LIBRARY_PATH value is " << ld_library_path << std::endl;
    
    std::vector<std::string> path_list;
    boost::split(path_list, ld_library_path, [](char c){return c == ':';});
    
    std::cout << "Single paths are: \n";
    for(auto path : path_list)
    {
        std::cout << "---  " << path << std::endl;
    }
    
    
    std::string lib_name = argv[1];
    std::vector<boost::filesystem::path> found_items;
    
    
    for(auto path_string : path_list)
    {
        boost::filesystem::path current_path(path_string);
        if(!boost::filesystem::is_directory(current_path))
        {
            std::cout << "Error! Path \"" << path_string << "\" is not a directory" <<  std::endl;
            continue;
        }
        
        boost::filesystem::directory_iterator it_begin(current_path);
        boost::filesystem::directory_iterator it_end;
        
        for (auto it = it_begin; it != it_end; it++){ 
            if(it->path().filename() == lib_name)
            {
                found_items.push_back(it->path());
            }
        }
    }
    
    
    if(found_items.size() == 0)
    {
        std::cout << "Error! File NOT found" <<  std::endl;
    }
    else if(found_items.size() == 1)
    {
        std::cout << "Success! Found item " << found_items[0] <<  std::endl;
    }
    else
    {
        std::cout << "Warning! Found more than one item:" <<  std::endl;
        for(auto& item : found_items)
        {
            std::cout << "----- " << item << std::endl;
        }
    }
    
    
    
}
