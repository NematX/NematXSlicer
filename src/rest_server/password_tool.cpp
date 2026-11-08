// Standalone account-tool wrapper. Windows arguments are converted to UTF-8;
// the shared library implementation reads the password without terminal echo.
#include "rest_main.hpp"
#include <boost/nowide/args.hpp>

int main(int argc, char **argv)
{
    boost::nowide::args utf8_arguments(argc, argv);
    return Slic3r::Rest::rest_password_main(argc, argv);
}
