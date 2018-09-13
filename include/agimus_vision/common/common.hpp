#include <string>

bool strEndsWith( std::string str, std::string end)
{
    for( size_t i{ 0 } ; i <= end.size() ; ++i )
        if( str[ str.size() - end.size() + i ] != end[ i ] )
            return false;
    return true;
}
