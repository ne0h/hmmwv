#include <iostream>
#include <sstream>

#include "console.hpp"
#include "enginecontrol_common.hpp"

int main(int argc, char *argv[]) {

    if (argc < 2) {
        std::cerr << "You have to provide a command as argument" << std::endl;
        exit(EXIT_FAILURE);
    }

    // read input
    uint16_t cmd;   
    std::stringstream ss;
    ss << std::hex << argv[1];
    ss >> cmd;

    Console console("/dev/cu.wchusbserial1420");
    ssize_t result = console.send(cmd);
    std::cout << "Send " << result << " bytes" << std::endl;

    return 0;
}
