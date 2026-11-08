// Standalone wrapper: nowide obtains UTF-8 arguments from the Windows command line.
// Parsing and server lifecycle are shared with the application's REST launch mode.
#include "rest_main.hpp"
#include <boost/nowide/args.hpp>

int main(int argc, char **argv)
{
    boost::nowide::args utf8_arguments(argc, argv);
    return Slic3r::Rest::rest_server_main(argc, argv);
}
