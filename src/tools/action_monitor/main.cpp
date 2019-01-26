#include <QApplication>
#include "action_monitor.hpp"
#include "configuration.hpp"
#include "options/options.hpp"
#include "robot/humanoid.hpp"
#include "common.hpp"

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

    ROBOT->init(CONF->robot_file(), CONF->action_file(), CONF->offset_file());

    QApplication app(argc, argv);
    action_monitor foo;
    foo.show();
    return app.exec();
}
