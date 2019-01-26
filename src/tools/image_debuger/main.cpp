#include <QApplication>
#include "image_debuger.hpp"
#include "configuration.hpp"
#include "options/options.hpp"
#include "robot/humanoid.hpp"

using namespace std;
using namespace robot;

int main(int argc, char **argv)
{
    if (!OPTS->init(argc, argv))
    {
        std::cout << "options init failed\n";
        exit(1);
    }

    if (!CONF->init(OPTS->id()))
    {
        std::cout << "config init failed\n";
        exit(2);
    }

    QApplication app(argc, argv);
    image_debuger foo;
    foo.show();
    return app.exec();
}
