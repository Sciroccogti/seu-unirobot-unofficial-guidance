#include <QApplication>
#include "team_monitor.hpp"
#include "configuration.hpp"
#include "options/options.hpp"

using namespace std;

int main(int argc, char **argv)
{
    if (!CONF->init())
    {
        std::cout << "config init failed\n";
        exit(2);
    }

    QApplication app(argc, argv);
    team_monitor foo;
    foo.show();
    return app.exec();
}
