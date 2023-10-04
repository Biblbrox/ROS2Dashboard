#include "core/Application.hpp"
#include "core/Logger.hpp"

int main(int argc, char *argv[])
{
    using namespace ros2monitor;
    Logger::init(Category::debug, "log.txt");
    int code = 0;
    try {
        Application app(argc, argv);
        code = app.run();
        
    } catch (std::exception &e) {
        Logger::error(fmt::format("Unhandled exception: {}", e.what()));
        return EXIT_FAILURE;
    }

    return code;
}
